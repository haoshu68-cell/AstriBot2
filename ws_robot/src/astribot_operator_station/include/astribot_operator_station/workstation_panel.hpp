#pragma once
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <astribot_operator_msgs/srv/operator_command.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <QLabel>
#include <QDoubleSpinBox>
#include "astribot_operator_station/map_page.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <QPlainTextEdit>
#include <QTableWidget>
#include <QLineEdit>
#include <QTimer>
#include <QPushButton>
#include <nlohmann/json.hpp>
#include <thread>
namespace astribot_operator_station {
class WorkstationPanel : public rviz_common::Panel {
 Q_OBJECT
public:explicit WorkstationPanel(QWidget * parent=nullptr);~WorkstationPanel() override;
signals:void statusReceived(QString);void responseReceived(QString);void pointReceived(QString);
private:
 using Command=astribot_operator_msgs::srv::OperatorCommand;
 using Clock=std::chrono::steady_clock;
 rclcpp::Node::SharedPtr node_;rclcpp::executors::SingleThreadedExecutor executor_;std::thread thread_;
 rclcpp::Client<Command>::SharedPtr command_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
 rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr point_sub_;
 nlohmann::json state_;
 std::string boot_,lease_,last_command_,pending_command_,renew_command_,mapping_session_;Clock::time_point seen_{},renew_at_{},deadline_{},renew_deadline_{};
 bool pending_{false},renew_pending_{false};int64_t pending_id_{0},renew_id_{0};
 MapPage * maps_page_;std::string active_map_version_;
 QComboBox * arm_group_;QLabel * arm_status_;
 QLabel * summary_;QPlainTextEdit * log_;QTableWidget * points_;QLineEdit * frame_;QDoubleSpinBox * dwell_;
 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr preview_;int marker_count_{0};
 void preview();void routeFile(bool save);
 std::vector<QPushButton *> draft_buttons_;
 QTimer * timer_;std::map<std::string,QPushButton *> buttons_;
 void submit(const std::string &,nlohmann::json payload=nlohmann::json::object(),bool confirmation=true);
 void refresh();void replay();
};
}
