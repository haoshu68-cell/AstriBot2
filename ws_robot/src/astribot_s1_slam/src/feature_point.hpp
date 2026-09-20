#ifndef FEATURE_POINT_HPP
#define FEATURE_POINT_HPP

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <functional>

typedef pcl::PointXYZINormal PointType;
using namespace std;

// The S1 mapping contract is standard PointCloud2.  The first value is kept
// at zero because the deployed Mid-360 configuration already uses 0 for its
// lidar type; it no longer means Livox CustomMsg.
enum LID_TYPE{GENERIC_POINTCLOUD2, VELODYNE, OUSTER, HESAI, ROBOSENSE, TARTANAIR};

namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      // float intensity;
      float time;
      std::uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    // (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)

namespace ouster_ros 
{
  struct EIGEN_ALIGN16 Point 
  {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t  ring;
    // uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  // use std::uint32_t to avoid conflicting with pcl::uint32_t
  (std::uint32_t, t, t)
  // (std::uint16_t, reflectivity, reflectivity)
  // (std::uint8_t, ring, ring)
  // (std::uint16_t, ambient, ambient)
  // (std::uint32_t, range, range)
)

namespace xt32_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      double timestamp;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)


namespace rslidar_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      std::uint16_t ring;
      double timestamp;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

class Features
{
public:
  int lidar_type, point_filter_num;
  // Chassis-centered blind-zone exclusion volume: an axis-aligned BOX in
  // the chassis's own frame (not lidar-aligned — the chassis footprint is
  // rectangular and not centered on either lidar, so this needs blind_R
  // below to actually line up with it), matching the real rectangular
  // footprint far better than a circle/cylinder would. blind_half_x is
  // the half-extent along the chassis's forward/back axis (front-to-back
  // depth / 2), blind_half_y along its left/right axis (front-face width
  // / 2); blind_z_min/max are heights RELATIVE TO blind_center (the
  // chassis origin), not the ground — see General.blind_half_x/y and
  // blind_z_min/max in the yaml for the actual chassis dimensions and
  // ground-relative height numbers these encode.
  double blind_half_x = 1, blind_half_y = 1;
  double blind_z_min = -1e9, blind_z_max = 1e9;
  double omega_l = 3610;

  // Center of the blind-zone exclusion box, expressed in this lidar's own
  // raw sensor frame — same role as before. blind_R additionally rotates
  // a point (once re-centered on blind_center) from this lidar's own raw
  // axes into the chassis's own aligned axes, since the box's half-
  // extents are only meaningful along the chassis's actual forward/
  // left axes, not whichever way this lidar happens to be mounted. With
  // two lidars sharing this one Features instance, the caller
  // (pcl_handler/pcl_handler_back) sets blind_center/blind_R to that
  // lidar's own-frame position/orientation of the chassis before calling
  // process(), so the blind zone is really "too close to the chassis
  // body" rather than "too close to wherever this particular lidar
  // happens to be mounted".
  Eigen::Vector3d blind_center = Eigen::Vector3d::Zero();
  Eigen::Matrix3d blind_R = Eigen::Matrix3d::Identity();

  std::function<bool(const Eigen::Vector3d &)> robot_mask;

  inline bool point_in_blind(const PointType &p) const
  {
    Eigen::Vector3d d = blind_R * (Eigen::Vector3d(p.x, p.y, p.z) - blind_center);
    if(robot_mask && robot_mask(d)) return true;
    return std::fabs(d.x()) < blind_half_x && std::fabs(d.y()) < blind_half_y
           && d.z() > blind_z_min && d.z() < blind_z_max;
  }

  double process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    double t0 = stamp2sec(msg->header.stamp);
    switch(lidar_type)
    {
    case GENERIC_POINTCLOUD2:
      generic_pointcloud2_handler(msg, pl_full);
      break;

    case VELODYNE:
      velodyne_handler(msg, pl_full);
      break;

    case OUSTER:
      ouster_handler(msg, pl_full);
      break;

    case HESAI:
      hesai_handler(msg, pl_full);
      break;
    
    case ROBOSENSE:
      t0 = robosense_handler(msg, pl_full);
      break;
    
    case TARTANAIR:
      tartanair_handler(msg, pl_full);
      break;

    default:
      LOG_ERROR(LIDAR, "unknown lidar_type:{} — check General.lidar_type in config", lidar_type);
      exit(0);
    }

    return t0;
  }

  static const sensor_msgs::msg::PointField *find_field(
    const sensor_msgs::msg::PointCloud2 &msg, const std::string &name)
  {
    for(const auto &field : msg.fields)
      if(field.name == name && field.count == 1)
        return &field;
    return nullptr;
  }

  static bool read_scalar(
    const sensor_msgs::msg::PointCloud2 &msg, std::size_t offset,
    const sensor_msgs::msg::PointField &field, double &value)
  {
    std::size_t size = 0;
    switch(field.datatype)
    {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8: size = 1; break;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16: size = 2; break;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::FLOAT32: size = 4; break;
    case sensor_msgs::msg::PointField::FLOAT64: size = 8; break;
    default: return false;
    }
    if(field.count != 1 || field.offset > msg.point_step ||
       size > msg.point_step - field.offset ||
       offset > msg.data.size() || field.offset > msg.data.size() - offset ||
       size > msg.data.size() - offset - field.offset)
      return false;

    const std::uint8_t *ptr = msg.data.data() + offset + field.offset;
    switch(field.datatype)
    {
    case sensor_msgs::msg::PointField::INT8:
      value = *reinterpret_cast<const std::int8_t *>(ptr); return true;
    case sensor_msgs::msg::PointField::UINT8:
      value = *reinterpret_cast<const std::uint8_t *>(ptr); return true;
    case sensor_msgs::msg::PointField::INT16:
    {
      std::int16_t v; std::memcpy(&v, ptr, sizeof(v)); value = v; return true;
    }
    case sensor_msgs::msg::PointField::UINT16:
    {
      std::uint16_t v; std::memcpy(&v, ptr, sizeof(v)); value = v; return true;
    }
    case sensor_msgs::msg::PointField::INT32:
    {
      std::int32_t v; std::memcpy(&v, ptr, sizeof(v)); value = v; return true;
    }
    case sensor_msgs::msg::PointField::UINT32:
    {
      std::uint32_t v; std::memcpy(&v, ptr, sizeof(v)); value = v; return true;
    }
    case sensor_msgs::msg::PointField::FLOAT32:
    {
      float v; std::memcpy(&v, ptr, sizeof(v)); value = v; return true;
    }
    case sensor_msgs::msg::PointField::FLOAT64:
    {
      double v; std::memcpy(&v, ptr, sizeof(v)); value = v; return true;
    }
    default:
      return false;
    }
  }

  // Generic S1 input contract.  x/y/z and intensity are ordinary scalar
  // fields.  The preferred time field is `time` in relative seconds for
  // simulation.  Livox's standard PointCloud2 uses `timestamp` in absolute
  // nanoseconds, so it is normalized against header.stamp here.  A missing
  // time field leaves curvature at zero; the downstream point_notime
  // parameter must then be enabled for that sensor.
  void generic_pointcloud2_handler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
    pcl::PointCloud<PointType> &pl_full)
  {
    if(msg->is_bigendian || msg->point_step == 0 || msg->width == 0 ||
       msg->height == 0 || msg->row_step < msg->point_step * msg->width)
    {
      LOG_ERROR(LIDAR, "generic PointCloud2 layout invalid (bigendian:{} point_step:{} width:{} height:{} row_step:{})",
                msg->is_bigendian, msg->point_step, msg->width, msg->height, msg->row_step);
      return;
    }

    const auto *fx = find_field(*msg, "x");
    const auto *fy = find_field(*msg, "y");
    const auto *fz = find_field(*msg, "z");
    const auto *fi = find_field(*msg, "intensity");
    const auto *ftime = find_field(*msg, "time");
    const auto *ftimestamp = find_field(*msg, "timestamp");
    const auto *foffset = find_field(*msg, "offset_time");
    if(!fx || !fy || !fz)
    {
      LOG_ERROR(LIDAR, "generic PointCloud2 requires x/y/z fields; received {} fields",
                msg->fields.size());
      return;
    }

    const std::size_t count = static_cast<std::size_t>(msg->width) * msg->height;
    if(msg->data.size() < static_cast<std::size_t>(msg->row_step) * msg->height)
    {
      LOG_ERROR(LIDAR, "generic PointCloud2 data truncated (bytes:{} expected:{})",
                msg->data.size(), static_cast<std::size_t>(msg->row_step) * msg->height);
      return;
    }
    pl_full.reserve(count);

    const double header_ns = static_cast<double>(msg->header.stamp.sec) * 1e9 +
      static_cast<double>(msg->header.stamp.nanosec);
    const int stride = std::max(1, point_filter_num);
    for(std::size_t index = 0; index < count; ++index)
    {
      if(index % static_cast<std::size_t>(stride) != 0)
        continue;
      const std::size_t row = index / msg->width;
      const std::size_t col = index % msg->width;
      const std::size_t base = row * msg->row_step + col * msg->point_step;
      PointType point;
      double x = 0.0, y = 0.0, z = 0.0, intensity = 0.0;
      if(!read_scalar(*msg, base, *fx, x) || !read_scalar(*msg, base, *fy, y) ||
         !read_scalar(*msg, base, *fz, z))
        continue;
      if(fi)
        (void)read_scalar(*msg, base, *fi, intensity);
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y);
      point.z = static_cast<float>(z);
      point.intensity = static_cast<float>(intensity);
      point.curvature = 0.0;

      double raw_time = 0.0;
      if(ftime && read_scalar(*msg, base, *ftime, raw_time))
        point.curvature = raw_time;
      else if(ftimestamp && read_scalar(*msg, base, *ftimestamp, raw_time))
        point.curvature = (raw_time - header_ns) * 1e-9;
      else if(foffset && read_scalar(*msg, base, *foffset, raw_time))
        point.curvature = raw_time * 1e-9;

      if(!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
         !std::isfinite(point.curvature) || point_in_blind(point))
        continue;
      pl_full.push_back(point);
    }
  }

  void velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.size();
    if(plsize == 0) return;
    if(pl_orig.back().time > 0.01 && pl_orig.back().time < 0.12)
    {
      // for(velodyne_ros::Point &iter : pl_orig.points)
      for(int i=0; i<plsize; i++)
      {
        velodyne_ros::Point &iter = pl_orig[i];
        PointType ap;
        ap.x = iter.x; ap.y = iter.y; ap.z = iter.z;
        
        // ap.intensity = iter.intensity;
        // ap.curvature = iter.time * 1e-3; // ms
        // ap.curvature = iter.time * 1e-6;
        ap.curvature = iter.time;

        if(i % point_filter_num == 0)
        {
          if(!point_in_blind(ap))
          {
            pl_full.push_back(ap);
          }
        }
      }

    }
    else
    {
      // lidar clockwise rotate
      bool first_point = true;
      double yaw_first = 0;
      double yaw_last = 0;
      double yaw_bias = 0;
      int cool = 0;
      float max_ang = 0;
      for(int i=0; i<plsize; i++)
      {
        cool--;
        velodyne_ros::Point &iter = pl_orig[i];
        PointType ap;
        ap.x = iter.x; ap.y = iter.y; ap.z = iter.z;

        if(fabs(ap.x) < 0.1)
          continue;
        
        double yaw_angle = atan2(ap.y, ap.x) * 57.2957 - yaw_bias;
        if(first_point)
        {
          yaw_first = yaw_angle;
          yaw_last  = yaw_angle;
          first_point = false;
        }

        if(point_in_blind(ap))
          continue;

        if(yaw_angle - yaw_last > 180 && cool <= 0)
        {
          yaw_bias += 360; yaw_angle-= 360; cool = 1000;
        }

        if(fabs(yaw_angle - yaw_last) > 180)
        {
          yaw_angle += 360;
        }

        ap.curvature = (yaw_first - yaw_angle) / omega_l;
        yaw_last = yaw_angle;

        if(ap.curvature > max_ang)
          max_ang = ap.curvature;

        if(ap.curvature >= 0 && ap.curvature < 0.1)
          if(i % point_filter_num == 0)
            pl_full.push_back(ap);
      }

      // printf("maxang: %f\n", max_ang);
    }

  }

  void ouster_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    for(int i=0; i<plsize; i++)
    {
      PointType ap;
      ap.x = pl_orig.points[i].x;
      ap.y = pl_orig.points[i].y;
      ap.z = pl_orig.points[i].z;
      ap.intensity = pl_orig[i].intensity;
      // ap.curvature = pl_orig[i].t / float(1e6); // ms
      ap.curvature = pl_orig[i].t / float(1e9); // s

      if(i % point_filter_num == 0)
      {
        if(!point_in_blind(ap))
        {
          pl_full.points.push_back(ap);
        }
      }

    }

  }

  void hesai_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, pcl::PointCloud<PointType> &pl_full)
  { 
    pcl::PointCloud<xt32_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    double time_head = pl_orig.points[0].timestamp;
    for(int i=0; i<plsize; i++)
    {
      PointType added_pt;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = (pl_orig.points[i].timestamp - time_head);

      if (i % point_filter_num == 0)
      {
        if (!point_in_blind(added_pt))
        {
          pl_full.points.push_back(added_pt);
        }
      }


    }

  }

  double robosense_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<rslidar_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    double t0 = pl_orig[0].timestamp;
    for(int i=0; i<plsize; i++)
    {
      PointType ap;
      ap.x = pl_orig.points[i].x;
      ap.y = pl_orig.points[i].y;
      ap.z = pl_orig.points[i].z;
      ap.intensity = pl_orig.points[i].intensity;
      // ap.curvature = (pl_orig[i].timestamp - t0) * float(1e3); //
      ap.curvature = (pl_orig[i].timestamp - t0);

      if(i % point_filter_num == 0)
      {
        if(!point_in_blind(ap))
        {
          pl_full.points.push_back(ap);
        }
      }

    }

    return t0;
  }

  void tartanair_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<pcl::PointXYZ> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    pl_full.reserve(pl_orig.size());

    PointType pp; pp.curvature = 0;
    for(pcl::PointXYZ &ap: pl_orig.points)
    {
      pp.x = ap.x;
      pp.y = ap.y;
      pp.z = ap.z; 
      pl_full.push_back(pp);
    }

    return;
  }

};

#endif
