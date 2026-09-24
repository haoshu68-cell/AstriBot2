#include <astribot_s1_manipulation_perception/single_box_request_source.hpp>
#include "single_box_source_detail.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nlohmann/json.hpp>
#include <openssl/sha.h>
#include <opencv2/imgproc.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace astribot::perception_planning {
namespace detail {
void require(bool value,const char* reason) {if(!value)throw std::runtime_error(reason);}
int64_t stamp(const builtin_interfaces::msg::Time& t) {
  require(t.sec>=0 && t.nanosec<1000000000u,"INVALID_SENSOR_STAMP");
  return int64_t(t.sec)*1000000000LL+t.nanosec;
}
std::chrono::steady_clock::time_point deadline(const SensorReceipt& receipt,int64_t until) {
  require(receipt.ros_ns>0&&receipt.steady.time_since_epoch().count()!=0,"SENSOR_RECEIPT_REQUIRED");
  return receipt.steady+std::chrono::nanoseconds(until-receipt.ros_ns);
}
bool fresh(const builtin_interfaces::msg::Time& time,const SensorReceipt& received,int64_t now,int64_t age) {
  const auto sample=stamp(time);
  return sample>0&&sample<=now&&received.ros_ns>=sample&&received.ros_ns<=now&&now-sample<=age&&
    std::chrono::steady_clock::now()<deadline(received,sample+age);
}
void health(const RgbdFrame& f,const Context& c,int64_t now) {
  const auto& h=f.camera;const auto& p=f.projection;
  require(h.valid&&fresh(h.header.stamp,f.camera_received,now,500000000LL)&&
    fresh(h.capture_stamp,f.camera_capture_received,now,500000000LL)&&stamp(h.valid_until)>now&&
    std::chrono::steady_clock::now()<deadline(f.camera_capture_received,stamp(h.valid_until)),"CAMERA_HEALTH_INVALID");
  require(!h.source_epoch.empty()&&h.source_epoch==c.source_epoch&&h.camera_id==c.camera_id&&
    h.frame_id==c.optical_frame&&h.header.frame_id==c.optical_frame&&
    h.calibration_revision>0&&h.calibration_revision==c.calibration_revision,"CAMERA_CONTEXT_MISMATCH");
  require(p.valid&&fresh(p.header.stamp,f.projection_received,now,250000000LL)&&
    fresh(p.capture_stamp,f.projection_capture_received,now,250000000LL)&&stamp(p.valid_until)>now&&
    std::chrono::steady_clock::now()<deadline(f.projection_capture_received,stamp(p.valid_until)),
    "PROJECTION_HEALTH_INVALID");
  require(!p.processing_epoch.empty()&&p.processing_epoch==c.processing_epoch&&p.camera_id==c.camera_id&&
    p.header.frame_id==c.optical_frame&&stamp(p.epoch_first_capture_stamp)>0,"PROJECTION_CONTEXT_MISMATCH");
}
}

std::string camera_info_revision(const sensor_msgs::msg::CameraInfo& i) {
  const nlohmann::json data={{"frame",i.header.frame_id},{"width",i.width},{"height",i.height},
    {"K",i.k},{"D",i.d},{"R",i.r},{"P",i.p},{"distortion_model",i.distortion_model},
    {"binning_x",i.binning_x},{"binning_y",i.binning_y},
    {"roi",{i.roi.x_offset,i.roi.y_offset,i.roi.width,i.roi.height,i.roi.do_rectify}}};
  const auto text=data.dump();unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(text.data()),text.size(),digest);
  std::ostringstream out;out<<std::hex<<std::setfill('0');
  for(auto byte:digest)out<<std::setw(2)<<int(byte);
  return out.str();
}

Request single_box_request(Request task,const SingleBoxFixture& fixture,const RgbdFrame& frame,
                           const geometry_msgs::msg::TransformStamped& tf,
                           const geometry_msgs::msg::TransformStamped& station_tf,int64_t now) {
  using detail::require;using detail::stamp;
  require(frame.color&&frame.points&&frame.info,"RGBD_FRAME_MISSING");
  const auto& rgb=*frame.color;const auto& cloud=*frame.points;const auto& info=*frame.info;
  const auto sample=stamp(rgb.header.stamp);
  require(sample>0&&sample<=now&&now-sample<=250000000LL,"RGBD_CAPTURE_STALE_OR_FUTURE");
  require(rgb.header.stamp==cloud.header.stamp&&rgb.header.stamp==info.header.stamp,"CAPTURE_STAMP_MISMATCH");
  require(detail::fresh(rgb.header.stamp,frame.color_received,now,250000000LL)&&
    detail::fresh(cloud.header.stamp,frame.points_received,now,250000000LL)&&
    detail::fresh(info.header.stamp,frame.info_received,now,250000000LL),"RGBD_RECEIVE_STALE");
  const auto& c=task.context;
  require(!c.optical_frame.empty()&&rgb.header.frame_id==c.optical_frame&&
    cloud.header.frame_id==c.optical_frame&&info.header.frame_id==c.optical_frame,"RGBD_FRAME_MISMATCH");
  detail::health(frame,c,now);
  require(sample>=stamp(frame.projection.epoch_first_capture_stamp)&&sample<=stamp(frame.projection.capture_stamp),
    "PROJECTION_CONTEXT_MISMATCH");
  require(!fixture.station_id.empty()&&fixture.confirmed_instances==1&&
    fixture.object_instance==c.object_instance&&fixture.identity_revision==c.identity_revision&&
    !fixture.identity_revision.empty(),"FIXTURE_IDENTITY_REQUIRED");
  require(stamp(fixture.issued_at)>0&&stamp(fixture.issued_at)<=sample&&stamp(fixture.valid_until)>now,
    "FIXTURE_WINDOW_INVALID");
  require(!fixture.station_frame.empty()&&!fixture.region_revision.empty()&&
    fixture.region_revision==c.station_region_revision,"STATION_REGION_REQUIRED");
  const std::array<double,3> low={fixture.region_min_m.x,fixture.region_min_m.y,fixture.region_min_m.z};
  const std::array<double,3> high={fixture.region_max_m.x,fixture.region_max_m.y,fixture.region_max_m.z};
  for(int i=0;i<3;++i)require(std::isfinite(low[i])&&std::isfinite(high[i])&&low[i]<high[i],"STATION_REGION_INVALID");
  require((rgb.encoding=="rgb8"||rgb.encoding=="bgr8")&&!rgb.is_bigendian&&rgb.width>0&&rgb.height>0&&
    uint64_t(rgb.width)*rgb.height<=4194304&&uint64_t(rgb.step)>=uint64_t(rgb.width)*3&&
    uint64_t(rgb.step)*rgb.height==rgb.data.size(),"RGB_LAYOUT_INVALID");
  require(info.width==rgb.width&&info.height==rgb.height,"RGBD_RESOLUTION_MISMATCH");
  require(std::all_of(info.k.begin(),info.k.end(),[](double v){return std::isfinite(v);})&&
    std::all_of(info.r.begin(),info.r.end(),[](double v){return std::isfinite(v);})&&
    std::all_of(info.p.begin(),info.p.end(),[](double v){return std::isfinite(v);})&&
    info.k[0]>0&&info.k[4]>0&&info.k[1]==0&&info.k[3]==0&&info.k[6]==0&&info.k[7]==0&&info.k[8]==1&&
    std::all_of(info.d.begin(),info.d.end(),[](double v){return v==0;})&&
    info.binning_x<=1&&info.binning_y<=1&&info.roi.x_offset==0&&info.roi.y_offset==0&&
    (info.roi.width==0||info.roi.width==info.width)&&(info.roi.height==0||info.roi.height==info.height),
    "RECTIFIED_PINHOLE_REQUIRED");
  require(!c.camera_info_revision.empty()&&c.camera_info_revision==camera_info_revision(info),"CAMERA_INFO_REVISION_MISMATCH");
  const uint64_t count=uint64_t(rgb.width)*rgb.height;
  require(cloud.height==1&&cloud.width==count&&!cloud.is_bigendian&&cloud.point_step==16&&
    cloud.row_step==count*16&&cloud.data.size()==count*16&&cloud.fields.size()==3,"PROJECTOR_LAYOUT_REQUIRED");
  for(unsigned i=0;i<3;++i) {
    const auto& field=cloud.fields[i];
    require(field.name==std::string(1,"xyz"[i])&&field.offset==i*4&&field.count==1&&
      field.datatype==sensor_msgs::msg::PointField::FLOAT32,"PROJECTOR_LAYOUT_REQUIRED");
  }
  const auto& q=tf.transform.rotation;const auto& t=tf.transform.translation;
  require(tf.header.frame_id=="astribot_torso_base"&&tf.child_frame_id==c.optical_frame&&tf.header.stamp==rgb.header.stamp&&
    std::isfinite(t.x)&&std::isfinite(t.y)&&std::isfinite(t.z)&&
    std::isfinite(q.x)&&std::isfinite(q.y)&&std::isfinite(q.z)&&std::isfinite(q.w)&&
    std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<1e-6,"CAPTURE_TF_REQUIRED");
  const auto& sq=station_tf.transform.rotation;const auto& st=station_tf.transform.translation;
  require(station_tf.header.frame_id==fixture.station_frame&&station_tf.child_frame_id==c.optical_frame&&
    station_tf.header.stamp==rgb.header.stamp&&std::isfinite(st.x)&&std::isfinite(st.y)&&std::isfinite(st.z)&&
    std::isfinite(sq.x)&&std::isfinite(sq.y)&&std::isfinite(sq.z)&&std::isfinite(sq.w)&&
    std::abs(sq.x*sq.x+sq.y*sq.y+sq.z*sq.z+sq.w*sq.w-1)<1e-6,"STATION_CAPTURE_TF_REQUIRED");
  tf2::Transform station;tf2::fromMsg(station_tf.transform,station);
  cv::Mat image(rgb.height,rgb.width,CV_8UC3,const_cast<uint8_t*>(rgb.data.data()),rgb.step),hsv,mask,labels,stats,centers;
  cv::cvtColor(image,hsv,rgb.encoding=="rgb8"?cv::COLOR_RGB2HSV:cv::COLOR_BGR2HSV);
  cv::inRange(hsv,cv::Scalar(5,100,45),cv::Scalar(30,255,255),mask);
  // Retain invalid depth color pixels whose rays intersect the workstation.
  // They remain in the 80% denominator; no invented XYZ enters the output.
  for(unsigned y=0;y<rgb.height;++y)for(unsigned x=0;x<rgb.width;++x) {
    if(!mask.at<uint8_t>(y,x))continue;
    std::array<float,3> p;std::memcpy(p.data(),cloud.data.data()+(uint64_t(y)*rgb.width+x)*16,12);
    bool inside=true;
    if(std::isfinite(p[0])&&std::isfinite(p[1])&&std::isfinite(p[2])&&p[2]>.08f&&p[2]<5.f) {
      require(std::abs(info.k[0]*p[0]/p[2]+info.k[2]-x)<.25&&
        std::abs(info.k[4]*p[1]/p[2]+info.k[5]-y)<.25,"PIXEL_REGISTRATION_MISMATCH");
      const auto measured=station*tf2::Vector3(p[0],p[1],p[2]);
      for(int a=0;a<3;++a)inside=inside&&measured[a]>=low[a]&&measured[a]<=high[a];
    } else {
      const auto ray=station.getBasis()*tf2::Vector3((x-info.k[2])/info.k[0],(y-info.k[5])/info.k[4],1.);
      const auto origin=station.getOrigin();double entry=.08,exit=5.;
      for(int a=0;a<3;++a) {
        if(std::abs(ray[a])<1e-12)inside=inside&&origin[a]>=low[a]&&origin[a]<=high[a];
        else {
          const double first=(low[a]-origin[a])/ray[a],second=(high[a]-origin[a])/ray[a];
          entry=std::max(entry,std::min(first,second));exit=std::min(exit,std::max(first,second));
        }
      }
      inside=inside&&entry<=exit;
    }
    if(!inside)mask.at<uint8_t>(y,x)=0;
  }
  const int components=cv::connectedComponentsWithStats(mask,labels,stats,centers,8,CV_32S);
  int selected=0;
  for(int i=1;i<components;++i)if(stats.at<int>(i,cv::CC_STAT_AREA)>=30) {
    require(selected==0,"SINGLE_COLOR_COMPONENT_REQUIRED");selected=i;
  }
  require(selected!=0,"SINGLE_COLOR_COMPONENT_REQUIRED");
  std::vector<std::array<float,3>> points;
  for(unsigned y=0;y<rgb.height;++y)for(unsigned x=0;x<rgb.width;++x) {
    if(labels.at<int>(y,x)!=selected)continue;
    std::array<float,3> p;std::memcpy(p.data(),cloud.data.data()+(uint64_t(y)*rgb.width+x)*16,12);
    if(!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2])||p[2]<=.08f||p[2]>=5.f)continue;
    require(std::abs(p[0])<=10&&std::abs(p[1])<=10,"TARGET_POINT_OUT_OF_RANGE");
    points.push_back(p);
  }
  require(double(points.size())/stats.at<int>(selected,cv::CC_STAT_AREA)>=.8,"TARGET_DEPTH_VALID_FRACTION");
  if(points.size()<2048)throw std::runtime_error("GRASPNET_POINTS_INSUFFICIENT:"+std::to_string(points.size()));
  task.object_cloud.header=rgb.header;task.object_cloud.height=1;task.object_cloud.is_dense=true;
  sensor_msgs::PointCloud2Modifier modifier(task.object_cloud);modifier.setPointCloud2FieldsByString(1,"xyz");
  const auto output_count=std::min(points.size(),std::size_t(12000));modifier.resize(output_count);
  for(std::size_t i=0;i<output_count;++i)
    std::memcpy(task.object_cloud.data.data()+i*task.object_cloud.point_step,points[i*points.size()/output_count].data(),12);
  task.capture_camera_info=info;task.base_from_camera=tf;task.capture_station_from_camera=station_tf;
  task.valid_until=rclcpp::Time(std::min<int64_t>(sample+5000000000LL,stamp(fixture.valid_until)),RCL_ROS_TIME);
  task.admission_deadline_steady=std::min({detail::deadline(frame.color_received,sample+500000000LL),
    detail::deadline(frame.points_received,sample+500000000LL),detail::deadline(frame.info_received,sample+500000000LL)});
  task.result_deadline_steady=std::min({detail::deadline(frame.color_received,stamp(task.valid_until)),
    detail::deadline(frame.points_received,stamp(task.valid_until)),detail::deadline(frame.info_received,stamp(task.valid_until))});
  task.detection_id=fixture.station_id+":"+c.processing_epoch+":"+std::to_string(sample);
  task.segmentation_source="single_instance_fixture";task.visible_instances=1;
  check_context(task,c,now,true);
  return task;
}
} // namespace astribot::perception_planning
