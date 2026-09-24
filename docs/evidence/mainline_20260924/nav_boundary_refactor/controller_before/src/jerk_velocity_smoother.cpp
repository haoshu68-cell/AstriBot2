#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <vector>
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ruckig/ruckig.hpp"

namespace astribot_s1_path_tracking
{
class JerkVelocitySmoother : public nav2_util::LifecycleNode
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
public:
  JerkVelocitySmoother() : nav2_util::LifecycleNode("velocity_smoother", "")
  {
    declare_parameter("smoothing_frequency", 50.0);
    declare_parameter("velocity_timeout", 0.3);
    declare_parameter("max_velocity", std::vector<double>{0.35,0.35,2.0});
    declare_parameter("min_velocity", std::vector<double>{-0.35,-0.35,-2.0});
    declare_parameter("normal_acceleration", std::vector<double>{0.25,0.25,0.6});
    declare_parameter("normal_jerk", std::vector<double>{0.5,0.5,1.2});
  }
protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    const auto frequency=get_parameter("smoothing_frequency").as_double();
    timeout_=get_parameter("velocity_timeout").as_double();
    auto hi=get_parameter("max_velocity").as_double_array();
    auto lo=get_parameter("min_velocity").as_double_array();
    auto accel=get_parameter("normal_acceleration").as_double_array();
    auto jerk=get_parameter("normal_jerk").as_double_array();
    if (!std::isfinite(frequency) || frequency<=0 || !std::isfinite(timeout_) || timeout_<=0 ||
      hi.size()!=3 || lo.size()!=3 || accel.size()!=3 || jerk.size()!=3) {return CallbackReturn::FAILURE;}
    for (size_t i=0;i<3;++i) {
      if (!std::isfinite(hi[i]) || !std::isfinite(lo[i]) || lo[i]>0 || hi[i]<0 ||
        !std::isfinite(accel[i]) || accel[i]<=0 || !std::isfinite(jerk[i]) || jerk[i]<=0)
      {return CallbackReturn::FAILURE;}
      maximum_[i]=hi[i];minimum_[i]=lo[i];
      input_.max_acceleration[i]=accel[i];input_.max_jerk[i]=jerk[i];
      input_.max_velocity[i]=std::max(std::abs(lo[i]),std::abs(hi[i]));
    }
    input_.control_interface=ruckig::ControlInterface::Velocity;
    input_.synchronization=ruckig::Synchronization::Phase;
    otg_=std::make_unique<ruckig::Ruckig<3>>(1./frequency);
    reset();
    publisher_=create_publisher<geometry_msgs::msg::Twist>("cmd_vel_smoothed",10);
    subscription_=create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (!active_) {return;}
        std::array<double,3> v{msg->linear.x,msg->linear.y,msg->angular.z};
        if (!std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);})) {
          stop();RCLCPP_ERROR(get_logger(),"SMOOTHER_INVALID_COMMAND");return;
        }
        for (size_t i=0;i<3;++i) {input_.target_velocity[i]=std::clamp(v[i],minimum_[i],maximum_[i]);}
        input_at_=now().seconds();have_input_=true;
      });
    timer_=create_wall_timer(std::chrono::duration<double>(1./frequency),[this](){tick();});
    RCLCPP_INFO(get_logger(),"JERK_SMOOTHER %.1fHz acceleration=(%.3f,%.3f,%.3f) jerk=(%.3f,%.3f,%.3f)",
      frequency,accel[0],accel[1],accel[2],jerk[0],jerk[1],jerk[2]);
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {publisher_->on_activate();reset();active_=true;tick_at_=now().seconds();createBond();return CallbackReturn::SUCCESS;}
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {stop();active_=false;publisher_->on_deactivate();destroyBond();return CallbackReturn::SUCCESS;}
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {active_=false;timer_.reset();subscription_.reset();publisher_.reset();otg_.reset();return CallbackReturn::SUCCESS;}
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {if (active_) {stop();}active_=false;destroyBond();return CallbackReturn::SUCCESS;}
private:
  void reset()
  {
    input_.current_position={0.,0.,0.};input_.current_velocity={0.,0.,0.};
    input_.current_acceleration={0.,0.,0.};input_.target_position={0.,0.,0.};
    input_.target_velocity={0.,0.,0.};input_.target_acceleration={0.,0.,0.};
    have_input_=false;if (otg_) {otg_->reset();}
  }
  void stop()
  {
    if (publisher_ && publisher_->is_activated()) {publisher_->publish(geometry_msgs::msg::Twist());}
    reset();
  }
  void tick()
  {
    if (!active_) {return;}
    const double current=now().seconds(), dt=current-tick_at_;tick_at_=current;
    if (dt<0 || dt>timeout_) {stop();return;}
    if (dt<=1e-6 || !have_input_) {return;}
    if (current-input_at_>timeout_) {stop();return;}
    otg_->delta_time=dt;
    const auto result=otg_->update(input_,output_);
    if (result<0) {stop();RCLCPP_ERROR(get_logger(),"SMOOTHER_TRAJECTORY_ERROR %d",result);return;}
    for (size_t i=0;i<3;++i) {
      if (input_.target_velocity[i]==0. && std::abs(output_.new_velocity[i])<1e-10 &&
        std::abs(output_.new_acceleration[i])<1e-9)
      {output_.new_velocity[i]=0.;output_.new_acceleration[i]=0.;}
    }
    geometry_msgs::msg::Twist msg;
    msg.linear.x=output_.new_velocity[0];msg.linear.y=output_.new_velocity[1];msg.angular.z=output_.new_velocity[2];
    publisher_->publish(msg);output_.pass_to_input(input_);
  }
  bool active_{false},have_input_{false};
  double timeout_{0.3},tick_at_{0.},input_at_{0.};
  std::array<double,3> minimum_{},maximum_{};
  ruckig::InputParameter<3> input_;
  ruckig::OutputParameter<3> output_;
  std::unique_ptr<ruckig::Ruckig<3>> otg_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};
}
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);auto node=std::make_shared<astribot_s1_path_tracking::JerkVelocitySmoother>();
  rclcpp::spin(node->get_node_base_interface());rclcpp::shutdown();return 0;
}
