#include "astribot_s1_path_tracking/envelope_guard.hpp"
#include "astribot_s1_path_tracking/corridor_refinement.hpp"
#include "astribot_s1_robot_geometry/narrow_translation.hpp"
#include "astribot_navigation_msgs/msg/passage_assessment.hpp"
#include "nav2_smac_planner/smac_planner_hybrid.hpp"
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include <type_traits>
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
template<class Search>
class ExactGoalPlannerBase : public Search {
  using Search::_costmap;
  using Search::_logger;
  class StartConnectionBlocked : public nav2_core::PlannerException {
  public:
    explicit StartConnectionBlocked(double heading)
      :nav2_core::PlannerException("START_CONNECTION_BLOCKED"),heading(heading) {}
    double heading;
  };
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    Search::configure(parent,name,tf,costmap); map_ros_=costmap;
    auto node=parent.lock();
    if constexpr (!std::is_same_v<Search,nav2_smac_planner::SmacPlanner2D>) {
      nav2_util::declare_parameter_if_not_declared(node,"planner_evaluation_only",rclcpp::ParameterValue(false));
      if(!node->get_parameter("planner_evaluation_only").as_bool()) {
        throw std::invalid_argument("SE2 planners require planner_evaluation_only; execution acceptance pending");
      }
    }
    geometry_guard_.configure(node,costmap,"planner");
    nav2_util::declare_parameter_if_not_declared(node,"navigation_policy_stage",rclcpp::ParameterValue("off"));
    const auto stage=node->get_parameter("navigation_policy_stage").as_string();
    corridor_direct_=stage=="p4" || stage=="p5";
    clock_=node->get_clock();
    if(corridor_direct_ && geometry_guard_.enabled()) {
      passage_sub_=node->create_subscription<astribot_navigation_msgs::msg::PassageAssessment>(
        "/navigation/passage_assessment",10,[this](astribot_navigation_msgs::msg::PassageAssessment::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(passage_mutex_);
          passage_=message;passage_received_=std::chrono::steady_clock::now();
        });
    }
    auto load=[&](const std::string & key,double fallback) {
      nav2_util::declare_parameter_if_not_declared(node,name+".quality."+key,rclcpp::ParameterValue(fallback));
      double v=node->get_parameter(name+".quality."+key).as_double();
      if (!std::isfinite(v) || v<=0) {throw std::invalid_argument("invalid path quality parameter: "+key);}
      return v;
    };
    max_k_=load("max_curvature",3.0); max_rate_=load("max_curvature_rate",12.0);
    displacement_=load("max_displacement",0.20);
    quality_pub_=rclcpp::create_publisher<std_msgs::msg::String>(node,"path_tracking/path_quality",10);
    plan_pub_=rclcpp::create_publisher<nav_msgs::msg::Path>(node,"plan",10);
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
  void cleanup() override {std::lock_guard<std::recursive_mutex> lock(planning_mutex_);passage_sub_.reset();passage_.reset();geometry_guard_.cleanup();candidate_service_.reset();valid_service_.reset();risk_pub_.reset();quality_pub_.reset();plan_pub_.reset();map_ros_.reset();Search::cleanup();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    std::lock_guard<std::recursive_mutex> lock(planning_mutex_);
    return plan(start,goal,true);
  }
private:
  nav_msgs::msg::Path plan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,bool diagnostics,bool check_start_connection=false)
  {
    auto emit=[&](const char * action,const PathQuality & before,const PathQuality & after) {
      if(diagnostics) {report(action,before,after);}
    };
    const auto connected=[&](nav_msgs::msg::Path output) {
      if(check_start_connection && geometry_guard_.enabled()) {
        nav_msgs::msg::Path turn;turn.header=output.header;
        turn.poses={start,start};
        turn.poses.back().pose.orientation=output.poses.front().pose.orientation;
        const int connection=collisionIndex(turn,0);
        if(connection==-1)
          throw nav2_core::PlannerException("PATH_CHECK_UNAVAILABLE: start connection unavailable");
        if(connection>=0)
          throw StartConnectionBlocked(tf2::getYaw(output.poses.front().pose.orientation));
      }
      return output;
    };
    if (!geometry_guard_.ready()) {throw nav2_core::PlannerException("ENVELOPE_V2_NOT_READY");}
    auto path=Search::createPlan(start,goal);
    if (path.poses.empty()) {return path;}
    if (pathDistance(path.poses.back(),goal)>std::sqrt(2.0)*_costmap->getResolution()) {
      throw nav2_core::PlannerException("Requested goal unavailable: planner returned a substitute endpoint");
    }
    if constexpr (std::is_same_v<Search,nav2_smac_planner::SmacPlanner2D>) {
      try {anchorGridPathStart(path,start,_costmap->getResolution());}
      catch(const std::invalid_argument & error) {
        throw nav2_core::PlannerException(std::string("PATH_START_UNAVAILABLE: ")+error.what());
      }
    }
    path.poses.back()=goal;path.poses.back().header=path.header;
    auto before=pathQuality(path);
    // Curvature and clearance are independent: Smac2D checks the centre, while
    // execution checks the swept footprint (including unknown cells).
    const int original_collision=collisionIndex(path,0);
    if (original_collision==-1) {
      throw nav2_core::PlannerException("PATH_CHECK_UNAVAILABLE: costmap unavailable during planning");
    }
    if constexpr (std::is_same_v<Search,nav2_smac_planner::SmacPlanner2D>) {
      if(auto translation=narrowStartPath(start,path)) {
        emit("narrow_start_translation",before,pathQuality(*translation));
        return std::move(*translation);
      }
    }
    if constexpr (!std::is_same_v<Search,nav2_smac_planner::SmacPlanner2D>) {
      // SE(2) primitive headings and reverse cusps must survive postprocessing.
      if(original_collision!=-2) {throw nav2_core::PlannerException("SE2_SWEEP_UNSAFE");}
      emit("se2_validated",before,before);return connected(std::move(path));
    }
    // Grid centres can introduce a lateral bend at an exact aligned endpoint.
    // For fixed V2 or corridor stages, consider the exact straight segment
    // within the existing displacement budget and validate its entire sweep.
    // Opposed endpoint headings and SE(2) primitive paths keep their semantics.
    if (geometry_guard_.enabled() || corridor_direct_) {
      const double dx=goal.pose.position.x-start.pose.position.x;
      const double dy=goal.pose.position.y-start.pose.position.y;
      const double length=std::hypot(dx,dy),heading=std::atan2(dy,dx);
      const auto aligned=[&](const geometry_msgs::msg::PoseStamped & p) {
        return directCandidateHeadingCompatible(tf2::getYaw(p.pose.orientation),heading,geometry_guard_.enabled());
      };
      bool nearby=length>1e-6 && length<100. && aligned(start) && aligned(goal);
      if (nearby) {
        for (const auto & p:path.poses) {
          const double x=p.pose.position.x-start.pose.position.x,y=p.pose.position.y-start.pose.position.y;
          const double f=std::clamp((x*dx+y*dy)/(length*length),0.,1.);
          if(std::hypot(x-f*dx,y-f*dy)>displacement_) {nearby=false;break;}
        }
      }
      if (nearby) {
        nav_msgs::msg::Path direct;direct.header=path.header;
        const int steps=std::max(1,int(std::ceil(length/(_costmap->getResolution()*.5))));
        for(int i=0;i<=steps;++i) {
          auto p=start;p.header=direct.header;const double f=double(i)/steps;
          p.pose.position.x+=f*dx;p.pose.position.y+=f*dy;
          p.pose.orientation.x=p.pose.orientation.y=0.;
          p.pose.orientation.z=std::sin(heading/2);p.pose.orientation.w=std::cos(heading/2);
          direct.poses.push_back(p);
        }
        direct.poses.front()=start;direct.poses.back()=goal;
        direct.poses.front().header=direct.poses.back().header=direct.header;
        if(collisionIndex(direct,0,true)==-2) {emit("aligned_direct_validated",before,pathQuality(direct));return connected(std::move(direct));}
      }
    }
    if (acceptableQuality(before,max_k_,max_rate_) && original_collision==-2) {
      emit("accepted",before,before);return connected(std::move(path));
    }
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
      if (collisionIndex(output,0)==-2) {emit("smoothed",before,after);return connected(std::move(output));}
    }
    if (collisionIndex(path,0)==-2) {emit("speed_limited",before,before);return connected(std::move(path));}
    emit("rejected",before,pathQuality(candidate));
    // Only a real rejected first connection with a clear remaining path is
    // recoverable here. Unrelated map, search or downstream failures retain
    // their ordinary failure semantics.
    if(check_start_connection && original_collision==0 && collisionIndex(path,1)==-2)
      throw StartConnectionBlocked(tf2::getYaw(path.poses.front().pose.orientation));
    throw nav2_core::PlannerException(
      "PATH_QUALITY_UNSAFE: candidate footprint touches obstacle, unknown or map boundary; segment="+
      std::to_string(original_collision));
  }
private:
  std::optional<nav_msgs::msg::Path> narrowStartPath(
      const geometry_msgs::msg::PoseStamped &start,const nav_msgs::msg::Path &path) {
    using namespace astribot_s1_robot_geometry;
    if(!corridor_direct_ || !geometry_guard_.enabled() || path.poses.size()<2)return std::nullopt;
    const double yaw=tf2::getYaw(start.pose.orientation);
    nav_msgs::msg::Path turn;turn.header=path.header;turn.poses={start,start};
    turn.poses.back().pose.orientation=path.poses.front().pose.orientation;
    if(std::abs(std::remainder(tf2::getYaw(turn.poses.back().pose.orientation)-yaw,2*M_PI))<=.05)return std::nullopt;
    astribot_navigation_msgs::msg::PassageAssessment::ConstSharedPtr passage;
    {
      std::lock_guard<std::mutex> lock(passage_mutex_);passage=passage_;
      if(!passage || passage->corridor_id.empty() || passage->width_m<=0. ||
         std::chrono::duration<double>(std::chrono::steady_clock::now()-passage_received_).count()>.3 ||
         (clock_->now()-rclcpp::Time(passage->header.stamp)).seconds()<0. ||
         (clock_->now()-rclcpp::Time(passage->header.stamp)).seconds()>.3)return std::nullopt;
    }
    const auto transform=map_ros_->getTfBuffer()->lookupTransform(path.header.frame_id,passage->header.frame_id,tf2::TimePointZero);
    auto point=[&](const geometry_msgs::msg::Point &p) {
      geometry_msgs::msg::PoseStamped a,b;a.header=passage->header;a.pose.position=p;a.pose.orientation.w=1.;
      tf2::doTransform(a,b,transform);return PassagePoint{b.pose.position.x,b.pose.position.y};
    };
    const auto entry=point(passage->entry),exit=point(passage->exit);
    if(!passageContains({start.pose.position.x,start.pose.position.y},entry,exit,passage->width_m))return std::nullopt;
    if(passage->in_place_rotation_allowed && collisionIndex(turn,0,true)==-2)return std::nullopt;
    const auto origin=passageCoordinates({start.pose.position.x,start.pose.position.y},entry,exit);
    const auto destination=passageCoordinates({path.poses.back().pose.position.x,path.poses.back().pose.position.y},entry,exit);
    if(!passage->bidirectional && destination[0]<origin[0])return std::nullopt;
    std::vector<PassagePoint> footprint;
    for(const auto &p:map_ros_->getRobotFootprint())footprint.push_back({p.x,p.y});
    auto reference=path;
    // The existing grid path may bow toward a wall between exactly aligned
    // endpoints. Use the same straight-candidate displacement budget here;
    // opposed body/travel headings are intentional for this maneuver.
    const auto &goal=path.poses.back().pose.position;
    const double dx=goal.x-start.pose.position.x,dy=goal.y-start.pose.position.y,length2=dx*dx+dy*dy;
    bool straight=length2>1e-12;
    for(const auto &p:path.poses) {
      if(!straight)break;
      const double x=p.pose.position.x-start.pose.position.x,y=p.pose.position.y-start.pose.position.y;
      const double f=std::clamp((x*dx+y*dy)/length2,0.,1.);
      straight=std::hypot(x-f*dx,y-f*dy)<=displacement_;
    }
    if(straight) {
      const double heading=std::atan2(dy,dx);
      for(auto &p:reference.poses) {
        const double x=p.pose.position.x-start.pose.position.x,y=p.pose.position.y-start.pose.position.y;
        const double f=std::clamp((x*dx+y*dy)/length2,0.,1.);
        p.pose.position.x=start.pose.position.x+f*dx;p.pose.position.y=start.pose.position.y+f*dy;
        p.pose.orientation.x=p.pose.orientation.y=0.;
        p.pose.orientation.z=std::sin(heading/2);p.pose.orientation.w=std::cos(heading/2);
      }
      reference.poses.back()=path.poses.back();
    }
    for(std::size_t i=1;i+1<reference.poses.size();++i) {
      const auto &at=reference.poses[i];
      if(!passageBodyOutside(footprint,{at.pose.position.x,at.pose.position.y,yaw},entry,exit,
          passage->boundary_tracking_margin_m))continue;
      auto candidate=reference;
      for(std::size_t j=0;j<=i;++j)candidate.poses[j].pose.orientation=start.pose.orientation;
      candidate.poses.insert(candidate.poses.begin()+i+1,at);
      if(collisionIndex(candidate,0,true)==-2)return candidate;
    }
    return std::nullopt;
  }
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
      if(req.mode==Candidate::Request::INITIAL) {
        nav_msgs::msg::Path goal_input;goal_input.header.frame_id=frame;goal_input.poses={req.goal};
        if(req.goal.header.frame_id!=frame || !validGeometry(goal_input,frame)) {
          res.reason="INVALID_GOAL";return;
        }
        auto output=plan(res.evaluated_start,req.goal,true,true);
        if(output.poses.empty()) {res.reason="EMPTY_PLAN";return;}
        const auto quality=pathQuality(output);
        res.curvature=quality.curvature;res.curvature_rate=quality.curvature_rate;
        output.header.stamp=res.evaluated_start.header.stamp;
        res.path=std::move(output);res.geometry_valid=true;res.reason="GEOMETRY_VALID";
        plan_pub_->publish(res.path);return;
      }
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
      const double initial=tf2::getYaw(res.evaluated_start.pose.orientation);
      const double turn=geometry_guard_.enabled() ? std::remainder(tf2::getYaw(output.poses.front().pose.orientation)-initial,2*M_PI) : 2*M_PI;
      for(int i=0;i<=64;++i) {
        auto pose=res.evaluated_start;tf2::Quaternion q;
        q.setRPY(0,0,initial+turn*i/64);pose.pose.orientation=tf2::toMsg(q);rotation.poses.push_back(pose);
      }
      if(collisionIndex(rotation,0)!=-2) {res.reason="TAKEOVER_ROTATION_COLLISION";return;}
      output.header.stamp=res.evaluated_start.header.stamp;
      res.path=std::move(output);res.geometry_valid=true;res.reason="GEOMETRY_VALID";
    } catch(const StartConnectionBlocked & error) {
      res.failure_code=Candidate::Response::START_CONNECTION_BLOCKED;
      res.required_start_heading=error.heading;res.reason=error.what();
      RCLCPP_WARN(_logger,"START_CONNECTION_BLOCKED start=(%.9f,%.9f,%.9f) required_heading=%.9f",
        res.evaluated_start.pose.position.x,res.evaluated_start.pose.position.y,
        tf2::getYaw(res.evaluated_start.pose.orientation),error.heading);
    } catch(const std::exception & error) {res.reason=std::string("PLANNING_FAILED: ")+error.what();}
  }
  int collisionIndex(const nav_msgs::msg::Path & path,size_t first,bool continuous=false)
  {
    if (!geometry_guard_.ready() || !map_ros_->isCurrent()) {return -1;}
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
        if (geometry_guard_.enabled() || continuous) {
          const double bound=(d+astribot_s1_robot_geometry::radius(footprint)*std::abs(dyaw))/(2*steps);
          if(astribot_s1_robot_geometry::collision(*_costmap,footprint,x,y,yaw+f*dyaw,bound)) {return int(i);}
          continue;
        }
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
  EnvelopeGuard geometry_guard_;
  bool corridor_direct_{false};
  rclcpp::Clock::SharedPtr clock_;
  std::mutex passage_mutex_;
  astribot_navigation_msgs::msg::PassageAssessment::ConstSharedPtr passage_;
  std::chrono::steady_clock::time_point passage_received_;
  rclcpp::Subscription<astribot_navigation_msgs::msg::PassageAssessment>::SharedPtr passage_sub_;
  std::recursive_mutex planning_mutex_;
  rclcpp::Service<Candidate>::SharedPtr candidate_service_;
  double max_k_{3},max_rate_{12},displacement_{.2};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_ros_;
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr valid_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr quality_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<astribot_navigation_msgs::msg::PathRisk>::SharedPtr risk_pub_;
};
class ExactGoalPlanner : public ExactGoalPlannerBase<nav2_smac_planner::SmacPlanner2D> {};
class ExactHybridPlanner : public ExactGoalPlannerBase<nav2_smac_planner::SmacPlannerHybrid> {};
class ExactLatticePlanner : public ExactGoalPlannerBase<nav2_smac_planner::SmacPlannerLattice> {};
}
PLUGINLIB_EXPORT_CLASS(astribot_s1_path_tracking::ExactGoalPlanner,nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(astribot_s1_path_tracking::ExactHybridPlanner,nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(astribot_s1_path_tracking::ExactLatticePlanner,nav2_core::GlobalPlanner)
