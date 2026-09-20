#include "astribot_operator_station/panel.hpp"
#include "astribot_operator_station/evidence.hpp"
#include "astribot_operator_station/exploration_controls.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDateTime>
#include <QFormLayout>
#include <QLabel>
#include <QMessageBox>
#include <QFileDialog>
#include <QDialog>
#include <QListWidget>
#include <QSplitter>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun>
#include <cmath>
#include <cstdlib>
#include <unistd.h>
namespace astribot_operator_station {
OperatorPanel::OperatorPanel(QWidget * parent):rviz_common::Panel(parent) {
 auto layout=new QVBoxLayout(this);
 layout->addWidget(new QLabel("Astribot 操作台 · P1 开发版（权限仍由部署隔离保证）"));
 layout->addWidget(new QLabel(QString("ROS domain: %1 · 目标由当前 DDS 图决定").arg(std::getenv("ROS_DOMAIN_ID")?std::getenv("ROS_DOMAIN_ID"):"0")));
 enable_=new QCheckBox("允许本面板提交控制请求");layout->addWidget(enable_);
 auto form=new QFormLayout;frame_=new QLineEdit("map");form->addRow("目标坐标系",frame_);
 auto number=[] {auto p=new QDoubleSpinBox;p->setDecimals(3);p->setRange(-10000,10000);return p;};
 x_=number();y_=number();yaw_=number();yaw_->setRange(-3.142,3.142);
 form->addRow("X (m)",x_);form->addRow("Y (m)",y_);form->addRow("Yaw (rad)",yaw_);layout->addLayout(form);
 send_=new QPushButton("确认并提交导航（operator 仲裁入口）");cancel_=new QPushButton("取消本面板的导航目标");
 layout->addWidget(send_);layout->addWidget(cancel_);
 static std::atomic<unsigned> instance{0};
 node_=std::make_shared<rclcpp::Node>("operator_panel_"+std::to_string(getpid())+"_"+std::to_string(instance++));
 nav_=rclcpp_action::create_client<Nav>(node_,"/navigate_to_pose");
 connect(send_,&QPushButton::clicked,this,[this]{navigate();});
 connect(cancel_,&QPushButton::clicked,this,[this]{cancel();});
 auto add=[&](const QString & label,const std::string & service){
   auto button=new QPushButton(label);auto client=node_->create_client<Trigger>(service);
   controls_.emplace_back(button,client);layout->addWidget(button);
   connect(button,&QPushButton::clicked,this,[this,client,service]{
     if(service=="/exploration_coordinator_node/cancel" && QMessageBox::question(this,
       "结束建图", "将取消探索并结束当前 SLAM 会话保存地图，之后不能直接恢复探索。继续？")!=QMessageBox::Yes)return;
     call(client);
   });
 };
 auto exploration=new ExplorationControls(node_,[this]{return enable_->isChecked();},this);
 layout->addWidget(exploration);
 connect(exploration,&ExplorationControls::report,this,&OperatorPanel::report);
 add("开始诊断采集","/diagnostics_recorder/start");
 add("停止并落盘","/diagnostics_recorder/stop");
 add("人工故障打点","/diagnostics_recorder/mark");
 add("请求关键参数快照","/diagnostics_recorder/snapshot");
 auto replay=new QPushButton("打开离线事件 / 参数回放");layout->addWidget(replay);
 connect(replay,&QPushButton::clicked,this,[this]{openReplay();});
 auto unavailable=new QLabel("未接入：地图事务 / 通用搬运下发 / 真机抓放 / 参数写入\n软件取消不是硬件急停；取消受理不代表设备已停稳。");
 unavailable->setWordWrap(true);layout->addWidget(unavailable);
 output_=new QPlainTextEdit;output_->setReadOnly(true);output_->setMaximumBlockCount(500);layout->addWidget(output_);
 connect(this,&OperatorPanel::report,output_,&QPlainTextEdit::appendPlainText,Qt::QueuedConnection);
 for(const auto & topic:{"/diagnostics_recorder/status","/exploration/state","/mapping_session/status"})
  subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(topic,rclcpp::QoS(1).best_effort(),[this,topic](std_msgs::msg::String::ConstSharedPtr m){emit report(QString::fromStdString(std::string(topic)+": "+m->data));}));
 timer_=new QTimer(this);timer_->setInterval(250);
 connect(timer_,&QTimer::timeout,this,[this]{
   send_->setEnabled(enable_->isChecked()&&!busy_&&nav_->action_server_is_ready());
   {std::lock_guard<std::mutex> lock(mutex_);cancel_->setEnabled(bool(handle_));}
   for(auto & p:controls_)p.first->setEnabled(enable_->isChecked()&&p.second->service_is_ready());
   std::lock_guard<std::mutex> lock(mutex_);
   for(auto it=pending_.begin();it!=pending_.end();) {
    if(std::chrono::steady_clock::now()>it->second.deadline){it->second.client->remove_pending_request(it->second.id);emit report("服务响应超时：操作结果未知，请核对状态后再操作");it=pending_.erase(it);}else ++it;
   }
 });timer_->start();
 executor_.add_node(node_);thread_=std::thread([this]{executor_.spin();});
}
OperatorPanel::~OperatorPanel(){timer_->stop();executor_.cancel();if(thread_.joinable())thread_.join();}
void OperatorPanel::navigate(){
 if(!enable_->isChecked()||busy_||!nav_->action_server_is_ready())return;
 if(frame_->text().trimmed().isEmpty())return;
 if(QMessageBox::question(this,"导航确认",QString("将通过 operator 优先级提交目标 (%1, %2, %3)，可能抢占 route/探索任务。继续？").arg(x_->value()).arg(y_->value()).arg(yaw_->value()))!=QMessageBox::Yes)return;
 busy_=true;
 Nav::Goal goal;goal.pose.header.frame_id=frame_->text().trimmed().toStdString();goal.pose.header.stamp=builtin_interfaces::msg::Time();  // Latest TF; panel clock need not match robot clock.
 goal.pose.pose.position.x=x_->value();goal.pose.pose.position.y=y_->value();
 goal.pose.pose.orientation.z=std::sin(yaw_->value()/2);goal.pose.pose.orientation.w=std::cos(yaw_->value()/2);
 rclcpp_action::Client<Nav>::SendGoalOptions options;
 options.goal_response_callback=[this](Handle::SharedPtr h){std::lock_guard<std::mutex> lock(mutex_);handle_=h;if(!h)busy_=false;emit report(h?"导航已受理，等待执行结果":"导航被拒绝");};
 options.result_callback=[this](const Handle::WrappedResult & r){std::lock_guard<std::mutex> lock(mutex_);handle_.reset();busy_=false;QString state="未知终态";
  switch(r.code){
    case rclcpp_action::ResultCode::SUCCEEDED:state="导航成功";break;
    case rclcpp_action::ResultCode::ABORTED:state="导航中止";break;
    case rclcpp_action::ResultCode::CANCELED:state="导航取消完成";break;
    default:break;
  }
  emit report(state+QString(" (result_code=%1)").arg(static_cast<int>(r.code)));};
 try {nav_->async_send_goal(goal,options);emit report("导航请求已发送；无响应时不自动重发");}
 catch(const std::exception & e){busy_=false;emit report(e.what());}
}
void OperatorPanel::cancel(){
 std::lock_guard<std::mutex> lock(mutex_);if(!handle_)return;
 try {nav_->async_cancel_goal(handle_,[this](auto response){emit report(response->goals_canceling.empty()?"取消未确认，请核对任务状态":"取消已受理，等待导航终态；不是停稳确认");});}
 catch(const std::exception & e){emit report(e.what());}
}
void OperatorPanel::call(rclcpp::Client<Trigger>::SharedPtr client){
 if(!enable_->isChecked()||!client->service_is_ready())return;
 std::lock_guard<std::mutex> lock(mutex_);
 for(const auto & p:pending_)if(p.second.client==client){emit report("同一服务请求尚未完成");return;}
 auto seq=++sequence_;
 try {
 auto f=client->async_send_request(std::make_shared<Trigger::Request>(),[this,seq](rclcpp::Client<Trigger>::SharedFuture future){
  std::lock_guard<std::mutex> guard(mutex_);if(!pending_.erase(seq))return;
  try{auto r=future.get();emit report(QString(r->success?"成功/已受理：":"拒绝：")+QString::fromStdString(r->message));}
  catch(const std::exception & e){emit report(e.what());}
 });pending_.emplace(seq,Pending{client,f.request_id,std::chrono::steady_clock::now()+std::chrono::seconds(5)});
 }catch(const std::exception & e){emit report(e.what());}
}
void OperatorPanel::openReplay(){
 auto file=QFileDialog::getOpenFileName(this,"打开离线事件文件",{},"Events (events.jsonl *.jsonl)");if(file.isEmpty())return;
 auto watcher=new QFutureWatcher<std::vector<Json>>(this);
 connect(watcher,&QFutureWatcher<std::vector<Json>>::finished,this,[this,watcher]{
  try {
   auto events=std::make_shared<std::vector<Json>>(watcher->result());
   auto dialog=new QDialog(this);dialog->setAttribute(Qt::WA_DeleteOnClose);dialog->setWindowTitle("OFFLINE · 事件与参数回放（不发布 ROS 数据）");dialog->resize(1000,650);
   auto layout=new QVBoxLayout(dialog);layout->addWidget(new QLabel("按接收顺序回看（每 0.5 s 一条，非原始时间倍率）；observed 不等于确认生效。不是 3D bag 播放。"));
   auto transport=new QHBoxLayout;auto previous=new QPushButton("上一事件");auto next=new QPushButton("下一事件");auto play=new QPushButton("顺序回看");auto pause=new QPushButton("暂停");
   for(auto button:{previous,next,play,pause})transport->addWidget(button);
   layout->addLayout(transport);
   auto split=new QSplitter(dialog);auto list=new QListWidget(split);auto detail=new QPlainTextEdit(split);detail->setReadOnly(true);layout->addWidget(split);
   for(const auto & e:*events)list->addItem(QString::number(e.at("sequence").get<qulonglong>())+" "+QDateTime::fromMSecsSinceEpoch(e.value("utc_ns",int64_t(0))/1000000,Qt::UTC).toString("HH:mm:ss.zzz")+" "+QString::fromStdString(e.value("kind","")));
   auto replay_timer=new QTimer(dialog);replay_timer->setInterval(500);
   connect(replay_timer,&QTimer::timeout,dialog,[list,replay_timer]{if(list->currentRow()+1<list->count())list->setCurrentRow(list->currentRow()+1);else replay_timer->stop();});
   connect(play,&QPushButton::clicked,dialog,[replay_timer]{replay_timer->start();});
   connect(pause,&QPushButton::clicked,dialog,[replay_timer]{replay_timer->stop();});
   connect(previous,&QPushButton::clicked,dialog,[list,replay_timer]{replay_timer->stop();if(list->currentRow()>0)list->setCurrentRow(list->currentRow()-1);});
   connect(next,&QPushButton::clicked,dialog,[list,replay_timer]{replay_timer->stop();if(list->currentRow()+1<list->count())list->setCurrentRow(list->currentRow()+1);});
   connect(list,&QListWidget::currentRowChanged,dialog,[events,detail](int row){if(row<0)return;const auto & e=events->at(row);detail->setPlainText(QString::fromStdString(e.dump(2)+"\n\n参数（按接收顺序）:\n"+parameters_at(*events,e.at("sequence").get<uint64_t>()).dump(2)));});
   dialog->show();if(!events->empty())list->setCurrentRow(0);
  }catch(const std::exception & e){emit report(QString("无法打开回放：")+e.what());}catch(...){emit report("无法打开回放文件");}
  watcher->deleteLater();
 });
 watcher->setFuture(QtConcurrent::run([file]{auto events=read_events(file.toStdString());if(events.size()>10000)throw std::runtime_error("MVP viewer limit: 10000 events; export smaller interval");return events;}));
}
}
PLUGINLIB_EXPORT_CLASS(astribot_operator_station::OperatorPanel,rviz_common::Panel)
