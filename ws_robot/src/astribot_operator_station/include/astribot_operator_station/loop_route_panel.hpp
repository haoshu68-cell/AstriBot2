#pragma once
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <astribot_operator_msgs/srv/start_loop_route.hpp>
#include <astribot_operator_msgs/srv/cancel_loop_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <QTableWidget>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QTimer>
#include <thread>
namespace astribot_operator_station {
class LoopRoutePanel : public rviz_common::Panel {
 Q_OBJECT
public:
 explicit LoopRoutePanel(QWidget * parent=nullptr);
 ~LoopRoutePanel() override;
signals:
 void receivedStatus(QString json);
 void receivedPoint(QString json);
 void startReply(QString json);
 void cancelReply(QString text);
 void report(QString text);
private:
 using Start=astribot_operator_msgs::srv::StartLoopRoute;
 using Cancel=astribot_operator_msgs::srv::CancelLoopRoute;
 rclcpp::Node::SharedPtr node_;
 rclcpp::executors::SingleThreadedExecutor executor_;
 std::thread thread_;
 rclcpp::Client<Start>::SharedPtr start_client_;
 rclcpp::Client<Cancel>::SharedPtr cancel_client_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
 rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr point_sub_;
 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
 QTableWidget * table_;QLineEdit * frame_;QDoubleSpinBox * dwell_;
 QCheckBox * enable_;QCheckBox * picking_;
 QLabel * status_;QPlainTextEdit * output_;QTimer * timer_;
 QPushButton * start_button_;QPushButton * cancel_button_;
 std::vector<QWidget *> editors_;
 std::string route_id_, boot_id_;
 bool cancel_pending_{false}, active_{false}, pending_{false}, uncertain_{false}, updating_{false};
 int current_index_{-1}, old_markers_{0};int64_t request_id_{0}, cancel_request_id_{0};
 Start::Request::SharedPtr pending_request_;
 std::chrono::steady_clock::time_point seen_{}, deadline_{}, cancel_deadline_{};
 void addPoint(double x,double y,double yaw);
 void reorder(int direction);
 void preview();
 void startRoute();void cancelRoute();
 void routeFile(bool save);
 void updateStatus(const QString & text);
 Start::Request::SharedPtr draft() const;
};
}
