// Copyright 2026 Astribot. Apache-2.0.
#include <cmath>
#include <limits>
#include <mutex>
#include "astribot_navigation_msgs/srv/plan_candidate.hpp"
#include "astribot_navigation_msgs/msg/path_risk.hpp"
#include "astribot_s1_path_tracking/path_risk_distance.hpp"
#include "astribot_s1_path_tracking/path_quality.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_smac_planner/smac_planner_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "std_msgs/msg/string.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/create_publisher.hpp"

namespace astribot_s1_path_tracking {
class ExactGoalPlanner : public nav2_smac_planner::SmacPlanner2D {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    SmacPlanner2D::configure(parent,name,tf,costmap); map_ros_=costmap;
    auto node=parent.lock();
    auto load=[&](const std::string & key,double fallback) {
      nav2_util::declare_parameter_if_not_declared(node,name+".quality."+key,rclcpp::ParameterValue(fallback));
      double v=node->get_parameter(name+".quality."+key).as_double();
      if (!std::isfinite(v) || v<=0) {throw std::invalid_argument("invalid path quality parameter: "+key);}
      return v;
    };
    max_k_=load("max_curvature",3.0); max_rate_=load("max_curvature_rate",12.0);
    displacement_=load("max_displacement",0.20);
    quality_pub_=rclcpp::create_publisher<std_msgs::msg::String>(node,"path_tracking/path_quality",10);
    risk_pub_=rclcpp::create_publisher<astribot_navigation_msgs::msg::PathRisk>(
      node,"navigation_policy/path_risk",rclcpp::QoS(1));
    candidate_service_=node->create_service<astribot_navigation_msgs::srv::PlanCandidate>(
      "path_tracking/plan_candidate", [this](
        astribot_navigation_msgs::srv::PlanCandidate::Request::SharedPtr req,
        astribot_navigation_msgs::srv::PlanCandidate::Response::SharedPtr res) {candidate(*req,*res);});
    valid_service_=node->create_service<nav2_msgs::srv::IsPathValid>("path_tracking/check_path",
      [this,clock=node->get_clock()](nav2_msgs::srv::IsPathValid::Request::SharedPtr req,
             nav2_msgs::srv::IsPathValid::Response::SharedPtr res) {
        astribot_navigation_msgs::msg::PathRisk evidence;
        evidence.stamp=clock->now();evidence.checked_path=req->path;
        evidence.blocked=true;
        geometry_msgs::msg::PoseStamped robot;
        if (!map_ros_->isCurrent() || !map_ros_->getRobotPose(robot) ||
            req->path.header.frame_id!=map_ros_->getGlobalFrameID() || req->path.poses.empty()) {
          res->is_valid=false; res->invalid_pose_indices.push_back(-1);
          risk_pub_->publish(evidence);return;
        }
        evidence.evaluated_start=robot;
        size_t first=0; double best=std::numeric_limits<double>::infinity();
        for (size_t i=0;i<req->path.poses.size();++i) {
          double d=pathDistance(robot,req->path.poses[i]);
          if (!std::isfinite(d)) {res->invalid_pose_indices.push_back(-1);risk_pub_->publish(evidence);return;}
          if (d<best) {best=d;first=i;}
        }
        int bad=collisionIndex(req->path,first);
        res->is_valid=bad==-2;
        if (!res->is_valid) {res->invalid_pose_indices.push_back(bad);}
        evidence.known=bad!=-1;evidence.blocked=!res->is_valid;
        if (bad>=0) {evidence.distance_m=pathRiskDistance(req->path,first,size_t(bad),best);}
        risk_pub_->publish(evidence);
      });
  }
  void cleanup() override {std::lock_guard<std::recursive_mutex> lock(planning_mutex_);candidate_service_.reset();valid_service_.reset();risk_pub_.reset();quality_pub_.reset();map_ros_.reset();SmacPlanner2D::cleanup();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    std::lock_guard<std::recursive_mutex> lock(planning_mutex_);
    return plan(start,goal,true);
  }
private:
  nav_msgs::msg::Path plan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,bool diagnostics)
  {
    auto emit=[&](const char * action,const PathQuality & before,const PathQuality & after) {
      if(diagnostics) {report(action,before,after);}
    };
    auto path=SmacPlanner2D::createPlan(start,goal);
    if (path.poses.empty()) {return path;}
    if (pathDistance(path.poses.back(),goal)>std::sqrt(2.0)*_costmap->getResolution()) {
      throw nav2_core::PlannerException("Requested goal unavailable: planner returned a substitute endpoint");
    }
    path.poses.back()=goal;path.poses.back().header=path.header;
    auto before=pathQuality(path);
    if (acceptableQuality(before,max_k_,max_rate_)) {emit("accepted",before,before);return path;}
    auto reference=resamplePath(path);auto candidate=reference;
    for (int iteration=1;iteration<=200;++iteration) {
      smoothPathStep(candidate,reference,displacement_);
      if (iteration%10) {continue;}
      auto after=pathQuality(candidate);
      if (!acceptableQuality(after,max_k_,max_rate_)) {continue;}
      auto output=restorePathDensity(candidate,path);
      after=pathQuality(output);
      if (!acceptableQuality(after,max_k_,max_rate_)) {continue;}
      for (size_t i=0;i+1<output.poses.size();++i) {
        auto & a=output.poses[i].pose; const auto & b=output.poses[i+1].pose;
        double yaw=std::atan2(b.position.y-a.position.y,b.position.x-a.position.x);
        a.orientation.x=a.orientation.y=0; a.orientation.z=std::sin(yaw/2);a.orientation.w=std::cos(yaw/2);
      }
      output.poses.front()=path.poses.front();output.poses.back()=path.poses.back();
      if (collisionIndex(output,0)==-2) {emit("smoothed",before,after);return output;}
    }
    if (collisionIndex(path,0)==-2) {emit("speed_limited",before,before);return path;}
    emit("rejected",before,pathQuality(candidate));
    throw nav2_core::PlannerException("PATH_QUALITY_UNSAFE: cannot smooth curvature within collision-free corridor");
  }
private:
  using Candidate=astribot_navigation_msgs::srv::PlanCandidate;
  static bool sameGoal(const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b) {
    return pathDistance(a,b)<1e-6 && std::abs(std::remainder(
      tf2::getYaw(a.pose.orientation)-tf2::getYaw(b.pose.orientation),2*M_PI))<1e-6;
  }
  static bool validGeometry(const nav_msgs::msg::Path & path,const std::string & frame) {
    if(path.header.frame_id!=frame || path.poses.empty() || path.poses.size()>20000) {return false;}
    for(const auto & point:path.poses) {
      const auto & p=point.pose.position;const auto & q=point.pose.orientation;
      double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if((!point.header.frame_id.empty() && point.header.frame_id!=frame) ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(norm) || std::abs(norm-1.)>1e-3 || std::abs(q.x)>1e-5 || std::abs(q.y)>1e-5) {
        return false;
      }
    }
    return true;
  }
  void candidate(const Candidate::Request & req,Candidate::Response & res) {
    std::lock_guard<std::recursive_mutex> lock(planning_mutex_);
    res.geometry_valid=false;
    try {
      if(!map_ros_ || !map_ros_->isCurrent() || !map_ros_->getRobotPose(res.evaluated_start)) {
        res.reason="WORLD_UNAVAILABLE";return;
      }
      const auto frame=map_ros_->getGlobalFrameID();
      if(req.goal.header.frame_id!=frame || !validGeometry(req.reference_path,frame) ||
        !sameGoal(req.reference_path.poses.back(),req.goal)) {
        res.reason="INVALID_REFERENCE_OR_GOAL";return;
      }
      nav_msgs::msg::Path output;
      size_t join=0;
      if(req.mode==Candidate::Request::VALIDATE) {
        output=req.candidate_path;
      } else if(req.mode==Candidate::Request::GLOBAL) {
        output=plan(res.evaluated_start,req.goal,false);
      } else if(req.mode==Candidate::Request::LOCAL) {
        if(!std::isfinite(req.rejoin_distance_m) || req.rejoin_distance_m<=0 ||
          !std::isfinite(req.max_local_deviation_m) || req.max_local_deviation_m<=0) {
          res.reason="INVALID_LOCAL_BOUNDS";return;
        }
        double best=std::numeric_limits<double>::infinity();
        for(size_t i=0;i<req.reference_path.poses.size();++i) {
          double d=pathDistance(res.evaluated_start,req.reference_path.poses[i]);
          if(d<best) {best=d;join=i;}
        }
        const size_t first=join;
        double length=0;
        while(join+1<req.reference_path.poses.size() && length<req.rejoin_distance_m) {
          length+=pathDistance(req.reference_path.poses[join],req.reference_path.poses[join+1]);++join;
        }
        // Near the goal the final pose is a valid rejoin. A coincident or
        // exhausted reference still cannot define a forward local detour.
        if(length<0.10 || join==first) {res.reason="NO_FORWARD_REJOIN";return;}
        output=plan(res.evaluated_start,req.reference_path.poses[join],false);
        for(const auto & pose:output.poses) {
          double deviation=std::numeric_limits<double>::infinity();
          for(size_t i=first;i<=join;++i) {
            deviation=std::min(deviation,pathDistance(pose,req.reference_path.poses[i]));
          }
          if(deviation>req.max_local_deviation_m) {res.reason="OUTSIDE_LOCAL_CORRIDOR";return;}
        }
        output.poses.insert(output.poses.end(),req.reference_path.poses.begin()+join+1,
                            req.reference_path.poses.end());
      } else {res.reason="INVALID_MODE";return;}
      if(!validGeometry(output,frame) ||
        !sameGoal(output.poses.back(),req.goal) ||
        pathDistance(output.poses.front(),res.evaluated_start)>.1) {
        res.reason="INVALID_ENDPOINT_OR_TAKEOVER";return;
      }
      auto quality=pathQuality(output);
      if(req.mode==Candidate::Request::LOCAL && !acceptableQuality(quality,max_k_,max_rate_)) {
        const auto reference=resamplePath(output);auto repaired=reference;
        for(int iteration=0;iteration<200;++iteration) {
          smoothPathStep(repaired,reference,displacement_);
          if(acceptableQuality(pathQuality(repaired),max_k_,max_rate_)) {
            auto dense=restorePathDensity(repaired,output);
            if(acceptableQuality(pathQuality(dense),max_k_,max_rate_)) {output=std::move(dense);break;}
          }
        }
        // The bounded smoother may alter the join; the final sweep below must
        // validate it, and the local corridor bound still applies afterwards.
        for(const auto & pose:output.poses) {
          double deviation=std::numeric_limits<double>::infinity();
          for(const auto & ref:req.reference_path.poses) {deviation=std::min(deviation,pathDistance(pose,ref));}
          if(deviation>req.max_local_deviation_m) {res.reason="SMOOTHED_LOCAL_CORRIDOR";return;}
        }
        quality=pathQuality(output);
      }
      res.curvature=quality.curvature;res.curvature_rate=quality.curvature_rate;
      if(!acceptableQuality(quality,max_k_,max_rate_)) {res.reason="CANDIDATE_CURVATURE";return;}
      // Even already-smooth candidates require a full footprint sweep, including the start.
      if(collisionIndex(output,0)!=-2) {res.reason="CANDIDATE_COLLISION_OR_UNKNOWN";return;}
      nav_msgs::msg::Path rotation;rotation.header=output.header;
      for(int i=0;i<=64;++i) {
        auto pose=res.evaluated_start;tf2::Quaternion q;
        q.setRPY(0,0,2*M_PI*i/64);pose.pose.orientation=tf2::toMsg(q);rotation.poses.push_back(pose);
      }
      if(collisionIndex(rotation,0)!=-2) {res.reason="TAKEOVER_ROTATION_COLLISION";return;}
      output.header.stamp=res.evaluated_start.header.stamp;
      res.path=std::move(output);res.geometry_valid=true;res.reason="GEOMETRY_VALID";
    } catch(const std::exception & error) {res.reason=std::string("PLANNING_FAILED: ")+error.what();}
  }
  int collisionIndex(const nav_msgs::msg::Path & path,size_t first)
  {
    if (!map_ros_->isCurrent()) {return -1;}
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
    auto footprint=map_ros_->getRobotFootprint();
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(_costmap);
    for (size_t i=first;i<path.poses.size();++i) {
      const auto & a=path.poses[i];const auto & b=path.poses[std::min(i+1,path.poses.size()-1)];
      double d=pathDistance(a,b),yaw=tf2::getYaw(a.pose.orientation);
      if (!std::isfinite(d) || d>10000 || !std::isfinite(yaw)) {return -1;}
      double dyaw=std::remainder(tf2::getYaw(b.pose.orientation)-yaw,2*M_PI);
      if (!std::isfinite(dyaw)) {return -1;}
      int steps=std::max({1,int(std::ceil(d/(_costmap->getResolution()*0.5))),int(std::ceil(std::abs(dyaw)/0.05))});
      for(int j=0;j<=steps;++j) {
        double f=double(j)/steps,x=a.pose.position.x+f*(b.pose.position.x-a.pose.position.x),
          y=a.pose.position.y+f*(b.pose.position.y-a.pose.position.y);
        unsigned int mx,my;
        double cost=checker.footprintCostAtPose(x,y,yaw+f*dyaw,footprint);
        if (!_costmap->worldToMap(x,y,mx,my) || cost<0 || cost>=254 || _costmap->getCost(mx,my)>=253) {return int(i);}
      }
    }
    return -2;
  }
  void report(const char * action,const PathQuality & before,const PathQuality & after) {
    std_msgs::msg::String msg;
    msg.data=std::string("{\"action\":\"")+action+"\",\"before_k\":"+std::to_string(before.curvature)+
      ",\"before_dk\":"+std::to_string(before.curvature_rate)+",\"after_k\":"+std::to_string(after.curvature)+
      ",\"after_dk\":"+std::to_string(after.curvature_rate)+"}";
    quality_pub_->publish(msg);RCLCPP_INFO(_logger,"PATH_QUALITY %s",msg.data.c_str());
  }
  std::recursive_mutex planning_mutex_;
  rclcpp::Service<Candidate>::SharedPtr candidate_service_;
  double max_k_{3},max_rate_{12},displacement_{.2};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_ros_;
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr valid_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr quality_pub_;
  rclcpp::Publisher<astribot_navigation_msgs::msg::PathRisk>::SharedPtr risk_pub_;
};
}
PLUGINLIB_EXPORT_CLASS(astribot_s1_path_tracking::ExactGoalPlanner,nav2_core::GlobalPlanner)
