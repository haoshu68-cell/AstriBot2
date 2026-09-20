#pragma once
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QTimer>
#include <atomic>
#include <mutex>
#include <thread>
namespace astribot_operator_station {
class OperatorPanel : public rviz_common::Panel {
 Q_OBJECT
public:
 explicit OperatorPanel(QWidget * parent=nullptr);
 ~OperatorPanel() override;
signals:
 void report(QString message);
private:
 using Nav=nav2_msgs::action::NavigateToPose;
 using Handle=rclcpp_action::ClientGoalHandle<Nav>;
 using Trigger=std_srvs::srv::Trigger;
 rclcpp::Node::SharedPtr node_;
 rclcpp::executors::SingleThreadedExecutor executor_;
 std::thread thread_;
 rclcpp_action::Client<Nav>::SharedPtr nav_;
 Handle::SharedPtr handle_;
 std::mutex mutex_;
 std::atomic<bool> busy_{false};
 QCheckBox * enable_;
 QLineEdit * frame_;
 QDoubleSpinBox * x_; QDoubleSpinBox * y_; QDoubleSpinBox * yaw_;
 QPushButton * send_; QPushButton * cancel_;
 QPlainTextEdit * output_;
 QTimer * timer_;
 std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
 std::vector<std::pair<QPushButton *,rclcpp::Client<Trigger>::SharedPtr>> controls_;
 struct Pending {rclcpp::Client<Trigger>::SharedPtr client; int64_t id; std::chrono::steady_clock::time_point deadline;};
 std::map<uint64_t,Pending> pending_;
 uint64_t sequence_{0};
 void navigate();void cancel();void call(rclcpp::Client<Trigger>::SharedPtr client);
 void openReplay();
};
}
