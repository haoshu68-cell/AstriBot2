/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Jon Binney, Ioan Sucan */

#include <cmath>
#include <astribot_s1_manipulation/observed_pointcloud_updater.hpp>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>

#include <memory>
#include <atomic>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <octomap_msgs/conversions.h>
#include <pluginlib/class_list_macros.hpp>

namespace astribot_s1_manipulation
{
// All instances in this process use a common epoch and order. Revision is
// assigned while the actual shared tree is write-locked, alongside serialization.
static const std::string map_epoch = boost::uuids::to_string(boost::uuids::random_generator()());
static std::atomic_uint64_t map_revision{0};
static const rclcpp::Logger LOGGER = rclcpp::get_logger("astribot.observed_pointcloud_updater");
ObservedPointCloudUpdater::ObservedPointCloudUpdater()
  : occupancy_map_monitor::OccupancyMapUpdater("ObservedPointCloudUpdater")
  , scale_(1.0)
  , padding_(0.0)
  , max_range_(std::numeric_limits<double>::infinity())
  , point_subsample_(1)
  , max_update_rate_(0)
  , point_cloud_subscriber_(nullptr)
  , point_cloud_filter_(nullptr)
{
}

ObservedPointCloudUpdater::~ObservedPointCloudUpdater()
{
  stopHelper();
}

bool ObservedPointCloudUpdater::setParams(const std::string& name_space)
{
  source_id_ = name_space;
  // This parameter is optional
  node_->get_parameter_or(name_space + ".ns", ns_, std::string());
  const bool configured = node_->get_parameter(name_space + ".point_cloud_topic", point_cloud_topic_) &&
         node_->get_parameter(name_space + ".max_range", max_range_) &&
         node_->get_parameter(name_space + ".padding_offset", padding_) &&
         node_->get_parameter(name_space + ".padding_scale", scale_) &&
         node_->get_parameter(name_space + ".point_subsample", point_subsample_) &&
         node_->get_parameter(name_space + ".max_update_rate", max_update_rate_) &&
         node_->get_parameter(name_space + ".filtered_cloud_topic", filtered_cloud_topic_);
  return configured && point_subsample_ > 0 && std::isfinite(max_range_) && max_range_ > 0 &&
         std::isfinite(scale_) && scale_ > 0 && std::isfinite(padding_) && padding_ >= 0 &&
         std::isfinite(max_update_rate_) && max_update_rate_ >= 0 && !point_cloud_topic_.empty();
}

bool ObservedPointCloudUpdater::initialize(const rclcpp::Node::SharedPtr& node)
{
  node_ = node;
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto create_timer_interface =
      std::make_shared<tf2_ros::CreateTimerROS>(node->get_node_base_interface(), node->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(create_timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  shape_mask_ = std::make_unique<point_containment_filter::ShapeMask>();
  shape_mask_->setTransformCallback(
      [this](ShapeHandle shape, Eigen::Isometry3d& tf) { return getShapeTransform(shape, tf); });

  return true;
}

void ObservedPointCloudUpdater::start()
{
  std::string prefix = "";
  if (!ns_.empty())
    prefix = ns_ + "/";

  observation_publisher_ = node_->create_publisher<astribot_transport_msgs::msg::ObservedOctomap>(
      "/moveit/observed_octomap", rclcpp::QoS(2).reliable());
  if (!filtered_cloud_topic_.empty())
  {
    filtered_cloud_publisher_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(prefix + filtered_cloud_topic_, rclcpp::SensorDataQoS());
  }

  if (point_cloud_subscriber_)
    return;

  rclcpp::SubscriptionOptions options;
  options.callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  /* subscribe to point cloud topic using tf filter*/
  point_cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(
      node_, point_cloud_topic_, rmw_qos_profile_sensor_data, options);
  if (tf_listener_ && tf_buffer_ && !monitor_->getMapFrame().empty())
  {
    point_cloud_filter_ = new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>(
        *point_cloud_subscriber_, *tf_buffer_, monitor_->getMapFrame(), 5, node_);
    point_cloud_filter_->registerCallback(
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud) { cloudMsgCallback(cloud); });
    RCLCPP_INFO(LOGGER, "Listening to '%s' using message filter with target frame '%s'", point_cloud_topic_.c_str(),
                point_cloud_filter_->getTargetFramesString().c_str());
  }
  else
  {
    point_cloud_subscriber_->registerCallback(
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud) { cloudMsgCallback(cloud); });
    RCLCPP_INFO(LOGGER, "Listening to '%s'", point_cloud_topic_.c_str());
  }
}

void ObservedPointCloudUpdater::stopHelper()
{
  delete point_cloud_filter_;
  delete point_cloud_subscriber_;
}

void ObservedPointCloudUpdater::stop()
{
  stopHelper();
  point_cloud_filter_ = nullptr;
  point_cloud_subscriber_ = nullptr;
}

ShapeHandle ObservedPointCloudUpdater::excludeShape(const shapes::ShapeConstPtr& shape)
{
  return shape_mask_->addShape(shape, scale_, padding_);
}

void ObservedPointCloudUpdater::forgetShape(ShapeHandle handle)
{
  shape_mask_->removeShape(handle);
}

bool ObservedPointCloudUpdater::getShapeTransform(ShapeHandle h, Eigen::Isometry3d& transform) const
{
  ShapeTransformCache::const_iterator it = transform_cache_.find(h);
  if (it != transform_cache_.end())
  {
    transform = it->second;
  }
  return it != transform_cache_.end();
}

void ObservedPointCloudUpdater::cloudMsgCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg)
{
  RCLCPP_DEBUG(LOGGER, "Received a new point cloud message");
  const auto start = std::chrono::steady_clock::now();
  astribot_transport_msgs::msg::ObservedOctomap observation;
  observation.header = cloud_msg->header;
  observation.callback_stamp = node_->now();
  observation.source_id = source_id_;
  observation.map_epoch = map_epoch;
  // PointCloud2 is an external byte buffer. The upstream iterators assume
  // packed native-endian float XYZ. Reject a different layout before reading it.
  if (cloud_msg->is_bigendian || cloud_msg->point_step < 12 ||
      cloud_msg->row_step != uint64_t(cloud_msg->width) * cloud_msg->point_step ||
      cloud_msg->data.size() != uint64_t(cloud_msg->height) * cloud_msg->row_step) {
    RCLCPP_ERROR(LOGGER, "POINT_CLOUD_LAYOUT_INVALID");
    return;
  }
  uint32_t xyz_offset = 0;
  for (const auto& coordinate : {std::pair<const char*, uint32_t>{"x", 0}, {"y", 4}, {"z", 8}}) {
    const auto found = std::find_if(cloud_msg->fields.begin(), cloud_msg->fields.end(),
        [&](const auto& field) { return field.name == coordinate.first; });
    if (found != cloud_msg->fields.end() && coordinate.second == 0) xyz_offset = found->offset;
    if (found == cloud_msg->fields.end() || found->offset != uint64_t(xyz_offset) + coordinate.second ||
        uint64_t(found->offset) + sizeof(float) > cloud_msg->point_step ||
        found->count != 1 || found->datatype != sensor_msgs::msg::PointField::FLOAT32) {
      RCLCPP_ERROR(LOGGER, "POINT_CLOUD_XYZ_LAYOUT_INVALID");
      return;
    }
  }

  if (max_update_rate_ > 0)
  {
    // ensure we are not updating the octomap representation too often
    if ((node_->now() - last_update_time_) <= rclcpp::Duration::from_seconds(1.0 / max_update_rate_))
      return;
    last_update_time_ = node_->now();
  }

  if (monitor_->getMapFrame().empty())
    monitor_->setMapFrame(cloud_msg->header.frame_id);

  /* get transform for cloud into map frame */
  tf2::Stamped<tf2::Transform> map_h_sensor;
  if (monitor_->getMapFrame() == cloud_msg->header.frame_id)
    map_h_sensor.setIdentity();
  else
  {
    if (tf_buffer_)
    {
      try
      {
        tf2::fromMsg(tf_buffer_->lookupTransform(monitor_->getMapFrame(), cloud_msg->header.frame_id,
                                                 cloud_msg->header.stamp),
                     map_h_sensor);
      }
      catch (tf2::TransformException& ex)
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Transform error of sensor data: " << ex.what() << "; quitting callback");
        return;
      }
    }
    else
      return;
  }

  observation.sensor_to_map = tf2::toMsg(static_cast<const tf2::Transform&>(map_h_sensor));

  /* compute sensor origin in map frame */
  const tf2::Vector3& sensor_origin_tf = map_h_sensor.getOrigin();
  octomap::point3d sensor_origin(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());
  Eigen::Vector3d sensor_origin_eigen(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());

  if (!updateTransformCache(cloud_msg->header.frame_id, cloud_msg->header.stamp))
    return;

  /* mask out points on the robot */
  shape_mask_->maskContainment(*cloud_msg, sensor_origin_eigen, 0.0, max_range_, mask_);

  octomap::KeySet free_cells, occupied_cells, model_cells, clip_cells, ray_free_cells;
  std::unique_ptr<sensor_msgs::msg::PointCloud2> filtered_cloud;

  // We only use these iterators if we are creating a filtered_cloud for
  // publishing. We cannot default construct these, so we use unique_ptr's
  // to defer construction
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_filtered_x;
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_filtered_y;
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_filtered_z;

  if (!filtered_cloud_topic_.empty())
  {
    filtered_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    filtered_cloud->header = cloud_msg->header;
    sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    pcd_modifier.resize(cloud_msg->width * cloud_msg->height);

    // we have created a filtered_out, so we can create the iterators now
    iter_filtered_x = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(*filtered_cloud, "x");
    iter_filtered_y = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(*filtered_cloud, "y");
    iter_filtered_z = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(*filtered_cloud, "z");
  }
  size_t filtered_cloud_size = 0;

  {
    const auto tree_lock = tree_->reading();
    /* do ray tracing to find which cells this point cloud indicates should be free, and which it indicates
     * should be occupied */
    for (unsigned int row = 0; row < cloud_msg->height; row += point_subsample_)
    {
      unsigned int row_c = row * cloud_msg->width;
      sensor_msgs::PointCloud2ConstIterator<float> pt_iter(*cloud_msg, "x");
      // set iterator to point at start of the current row
      pt_iter += row_c;

      for (unsigned int col = 0; col < cloud_msg->width; col += point_subsample_, pt_iter += point_subsample_)
      {
        /* Invalid depth does not establish either occupied or free cells. */
        if (std::isfinite(pt_iter[0]) && std::isfinite(pt_iter[1]) && std::isfinite(pt_iter[2]))
        {
          /* occupied cell at ray endpoint if ray is shorter than max range and this point
             isn't on a part of the robot*/
          if (mask_[row_c + col] == point_containment_filter::ShapeMask::INSIDE)
          {
            // transform to map frame
            tf2::Vector3 point_tf = map_h_sensor * tf2::Vector3(pt_iter[0], pt_iter[1], pt_iter[2]);
            model_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
          else if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
          {
            tf2::Vector3 clipped_point_tf =
                map_h_sensor * (tf2::Vector3(pt_iter[0], pt_iter[1], pt_iter[2]).normalize() * max_range_);
            clip_cells.insert(
                tree_->coordToKey(clipped_point_tf.getX(), clipped_point_tf.getY(), clipped_point_tf.getZ()));
          }
          else
          {
            tf2::Vector3 point_tf = map_h_sensor * tf2::Vector3(pt_iter[0], pt_iter[1], pt_iter[2]);
            occupied_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
            // build list of valid points if we want to publish them
            if (filtered_cloud)
            {
              **iter_filtered_x = pt_iter[0];
              **iter_filtered_y = pt_iter[1];
              **iter_filtered_z = pt_iter[2];
              ++filtered_cloud_size;
              ++*iter_filtered_x;
              ++*iter_filtered_y;
              ++*iter_filtered_z;
            }
          }
        }
      }
    }

    /* compute the free cells along each ray that ends at an occupied cell */
    for (const octomap::OcTreeKey& occupied_cell : occupied_cells)
      if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(occupied_cell), key_ray_))
        free_cells.insert(key_ray_.begin(), key_ray_.end());

    // Evidence uses measured obstacle/max-range rays only. Model clearing and
    // rays ending on the self-filter cannot manufacture free-space evidence.
    ray_free_cells = free_cells;

    /* compute the free cells along each ray that ends at a model cell */
    for (const octomap::OcTreeKey& model_cell : model_cells)
      if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(model_cell), key_ray_))
        free_cells.insert(key_ray_.begin(), key_ray_.end());

    /* compute the free cells along each ray that ends at a clipped cell */
    for (const octomap::OcTreeKey& clip_cell : clip_cells)
    {
      free_cells.insert(clip_cell);
      ray_free_cells.insert(clip_cell);
      if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(clip_cell), key_ray_)) {
        free_cells.insert(key_ray_.begin(), key_ray_.end());
        ray_free_cells.insert(key_ray_.begin(), key_ray_.end());
      }
    }
  }

  /* cells that overlap with the model are not occupied */
  for (const octomap::OcTreeKey& model_cell : model_cells)
    occupied_cells.erase(model_cell);

  /* occupied cells are not free */
  for (const octomap::OcTreeKey& occupied_cell : occupied_cells)
    free_cells.erase(occupied_cell);

  // Even if a model cell overlapped a measured ray, do not report it as
  // observed free. Nor does a ray override an occupied endpoint in this frame.
  for (const auto& key : model_cells) ray_free_cells.erase(key);
  for (const auto& key : occupied_cells) ray_free_cells.erase(key);

  {
    const auto tree_lock = tree_->writing();
    /* mark free cells only if not seen occupied in this cloud */
    for (const octomap::OcTreeKey& free_cell : free_cells)
      tree_->updateNode(free_cell, false);

    /* now mark all occupied cells */
    for (const octomap::OcTreeKey& occupied_cell : occupied_cells)
      tree_->updateNode(occupied_cell, true);

    // set the logodds to the minimum for the cells that are part of the model
    const float lg = tree_->getClampingThresMinLog() - tree_->getClampingThresMaxLog();
    for (const octomap::OcTreeKey& model_cell : model_cells)
      tree_->updateNode(model_cell, lg);
    observation.octomap.header.frame_id = monitor_->getMapFrame();
    observation.integrated_stamp = node_->now();
    observation.octomap.header.stamp = observation.integrated_stamp;
    if (!octomap_msgs::fullMapToMsg(*tree_, observation.octomap))
      throw std::runtime_error("OBSERVED_OCTOMAP_SERIALIZATION_FAILED");
    observation.map_revision = ++map_revision;
    observation.ray_free_keys.reserve(3 * ray_free_cells.size());
    for (const auto& key : ray_free_cells) {
      // Log-odds may still classify a newly traversed cell as occupied.
      if (tree_->isNodeOccupied(tree_->search(key))) continue;
      observation.ray_free_keys.insert(observation.ray_free_keys.end(), {key[0], key[1], key[2]});
    }
  }
  observation.processing_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  observation_publisher_->publish(observation);
  RCLCPP_DEBUG(LOGGER, "Processed point cloud in %lf ms", observation.processing_seconds * 1000.0);
  tree_->triggerUpdateCallback();

  if (filtered_cloud)
  {
    sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
    pcd_modifier.resize(filtered_cloud_size);
    filtered_cloud_publisher_->publish(*filtered_cloud);
  }
}
}  // namespace astribot_s1_manipulation

PLUGINLIB_EXPORT_CLASS(astribot_s1_manipulation::ObservedPointCloudUpdater, occupancy_map_monitor::OccupancyMapUpdater)
