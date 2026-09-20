#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <astribot_operator_msgs/srv/exploration_command.hpp>
#include <nlohmann/json.hpp>
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <functional>
namespace astribot_operator_station {
class ExplorationControls : public QWidget {
 Q_OBJECT
public:
 ExplorationControls(rclcpp::Node::SharedPtr node, std::function<bool()> enabled, QWidget * parent=nullptr);
 ~ExplorationControls() override;
signals:
 void statusReceived(QString source, QString data);
 void commandReply(QString id, QString boot, bool accepted, QString code, QString message);
 void retryReply(bool accepted, QString message);
 void report(QString message);
private:
 using Command=astribot_operator_msgs::srv::ExplorationCommand;
 using Clock=std::chrono::steady_clock;
 rclcpp::Node::SharedPtr node_;
 std::function<bool()> enabled_;
 rclcpp::Client<Command>::SharedPtr client_;
 rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr retry_;
 std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
 nlohmann::json exploration_, mapping_;
 Clock::time_point exploration_at_{}, mapping_at_{}, deadline_{}, retry_deadline_{};
 QLabel * summary_;
 QPushButton * pause_; QPushButton * resume_; QPushButton * cancel_; QPushButton * save_retry_; QPushButton * resolve_;
 QTimer * timer_;
 Command::Request::SharedPtr request_;
 bool pending_{false}, uncertain_{false}, retry_pending_{false};
 int64_t request_id_{0}, retry_id_{0};
 bool allowed(const std::string & operation) const;
 void refresh(); void command(const std::string & operation); void sendRequest();
};
}
