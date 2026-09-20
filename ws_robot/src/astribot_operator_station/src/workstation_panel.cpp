#include "astribot_operator_station/workstation_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QMessageBox>
#include <QFileDialog>
#include <QProcess>
#include <QStandardPaths>
#include <QUuid>
#include <QHeaderView>
#include <QSaveFile>
#include <cmath>
namespace astribot_operator_station {
using Json=nlohmann::json;
WorkstationPanel::WorkstationPanel(QWidget * parent):rviz_common::Panel(parent) {
 auto layout=new QVBoxLayout(this);summary_=new QLabel("等待机器人后端；默认观察模式");summary_->setTextFormat(Qt::PlainText);summary_->setWordWrap(true);layout->addWidget(summary_);
 auto tabs=new QTabWidget;layout->addWidget(tabs);
 auto nav=new QWidget;auto nav_layout=new QVBoxLayout(nav);tabs->addTab(nav,"导航与路线");
 frame_=new QLineEdit("map");nav_layout->addWidget(frame_);
 dwell_=new QDoubleSpinBox;dwell_->setRange(0,3600);dwell_->setValue(.5);dwell_->setSuffix(" s 到点等待");nav_layout->addWidget(dwell_);
 points_=new QTableWidget(0,3);points_->setObjectName("route_points");points_->setHorizontalHeaderLabels({"X (m)","Y (m)","Yaw (rad)"});points_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);nav_layout->addWidget(points_);
 auto add_button=[&](QVBoxLayout * parent,const QString & label,const std::string & op){auto b=new QPushButton(label);b->setEnabled(false);parent->addWidget(b);buttons_[op]=b;connect(b,&QPushButton::clicked,this,[this,op]{submit(op);});return b;};
 auto pick=new QPushButton("地图连续选点（拖动设置朝向）");nav_layout->addWidget(pick);
 connect(pick,&QPushButton::clicked,this,[this]{if(!getDisplayContext())return;auto manager=getDisplayContext()->getToolManager();
   for(int i=0;i<manager->numTools();++i)if(manager->getTool(i)->getClassId()=="astribot_operator_station/RoutePointTool"){manager->setCurrentTool(manager->getTool(i));return;}});
 auto edits=new QHBoxLayout;nav_layout->addLayout(edits);
 for(auto action:{"添加坐标","上移","下移","保存路线","加载路线"}) {
   auto b=new QPushButton(action);draft_buttons_.push_back(b);edits->addWidget(b);connect(b,&QPushButton::clicked,this,[this,action]{
     const auto name=QString::fromUtf8(action);
     if(name=="保存路线"||name=="加载路线"){routeFile(name=="保存路线");return;}
     if(name=="添加坐标"){if(points_->rowCount()>=200)return;int row=points_->rowCount();points_->insertRow(row);for(int c=0;c<3;++c)points_->setItem(row,c,new QTableWidgetItem("0"));points_->selectRow(row);}
     else {int row=points_->currentRow(),to=row+(name=="上移"?-1:1);if(row<0||to<0||to>=points_->rowCount())return;
       for(int c=0;c<3;++c){auto old=points_->takeItem(row,c);auto other=points_->takeItem(to,c);points_->setItem(row,c,other);points_->setItem(to,c,old);}points_->selectRow(to);}
     preview();
   });
 }
 auto remove=new QPushButton("删除选中点 / 无选中则清空");nav_layout->addWidget(remove);draft_buttons_.push_back(remove);connect(remove,&QPushButton::clicked,this,[this]{if(points_->currentRow()>=0)points_->removeRow(points_->currentRow());else points_->setRowCount(0);preview();});
 add_button(nav_layout,"导航到选中点（operator 优先级）","navigate");add_button(nav_layout,"取消本后端导航，等待终态","cancel_navigation");
 add_button(nav_layout,"按列表循环导航","start_route");add_button(nav_layout,"取消本后端循环路线","cancel_route");
 auto exploration=new QWidget;auto expl=new QVBoxLayout(exploration);tabs->addTab(exploration,"探索建图");
 add_button(expl,"开始新建图会话","new_mapping_session");add_button(expl,"暂停探索","explore_pause");add_button(expl,"恢复当前探索","explore_resume");
 add_button(expl,"结束探索并保存部分地图","explore_cancel_save");add_button(expl,"重试地图保存","retry_map");
 maps_page_=new MapPage;tabs->addTab(maps_page_,"地图与工位");
 connect(maps_page_,&MapPage::commandRequested,this,[this](QString op,QString payload){submit(op.toStdString(),Json::parse(payload.toStdString()));});
 connect(maps_page_,&MapPage::dockSelected,this,[this](QString p){emit pointReceived(p);});
 auto arm=new QWidget;auto arm_layout=new QVBoxLayout(arm);tabs->addTab(arm,"机械臂预览");
 arm_layout->addWidget(new QLabel("仅规划收臂位 transport_compact；计划有效期 30 秒。执行适配器尚未接入。"));
 arm_group_=new QComboBox;arm_group_->addItems({"arm_left","arm_right"});arm_layout->addWidget(arm_group_);
 add_button(arm_layout,"规划收臂轨迹（不会运动）","arm_plan");
 add_button(arm_layout,"丢弃计划 / 忽略迟到规划结果","arm_discard");
 auto execute_arm=new QPushButton("执行（安全执行适配器未接入）");execute_arm->setEnabled(false);arm_layout->addWidget(execute_arm);
 add_button(arm_layout,"开始仿真搬运任务（取货 → 导航 → 放置）","simulation_transport_start");
 add_button(arm_layout,"取消所属仿真搬运任务","simulation_transport_cancel");
 arm_status_=new QLabel("尚未连接");arm_status_->setWordWrap(true);arm_layout->addWidget(arm_status_);
 arm_layout->addWidget(new QLabel("3D 预览：MoveIt DisplayTrajectory 话题 /operator/arm_preview。需安装 MoveIt RViz 显示插件。\n抓放反馈来自 /transport/status；不将仿真附着解释为真实夹持成功。"));
 auto diagnostics=new QWidget;auto diag=new QVBoxLayout(diagnostics);tabs->addTab(diagnostics,"日志与回放");
 for(auto item:std::vector<std::pair<QString,std::string>>{{"开始记录","record_start"},{"停止记录并落盘","record_stop"},{"人工故障标记","record_mark"},{"关键参数快照","record_snapshot"}})add_button(diag,item.first,item.second);
 auto replay_button=new QPushButton("打开只读 3D / 参数回放（独立进程）");diag->addWidget(replay_button);connect(replay_button,&QPushButton::clicked,this,&WorkstationPanel::replay);
 add_button(layout,"申请控制权","acquire");add_button(layout,"释放控制权并请求停止所属任务","release");add_button(layout,"查询上次命令（不重新执行）","query");
 layout->addWidget(new QLabel("关闭工作站或断线后，租约到期会请求停止所属任务；软件取消不是急停。"));
 log_=new QPlainTextEdit;log_->setReadOnly(true);log_->setMaximumBlockCount(500);layout->addWidget(log_);
 node_=std::make_shared<rclcpp::Node>("workstation_"+QUuid::createUuid().toString(QUuid::WithoutBraces).remove('-').toStdString());
 preview_=node_->create_publisher<visualization_msgs::msg::MarkerArray>("/operator/route_preview",rclcpp::QoS(1).transient_local());
 connect(points_,&QTableWidget::cellChanged,this,[this]{preview();});
 command_=node_->create_client<Command>("/operator_backend/command");
 status_sub_=node_->create_subscription<std_msgs::msg::String>("/operator_backend/status",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){emit statusReceived(QString::fromStdString(m->data));});
 point_sub_=node_->create_subscription<geometry_msgs::msg::PoseStamped>("/operator/route_point",10,[this](geometry_msgs::msg::PoseStamped::ConstSharedPtr p){const auto & q=p->pose.orientation;
   emit pointReceived(QString::fromStdString(Json({{"frame",p->header.frame_id},{"x",p->pose.position.x},{"y",p->pose.position.y},{"yaw",std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}}).dump()));});
 connect(this,&WorkstationPanel::pointReceived,this,[this](QString text){try {if(!dwell_->isEnabled())return;auto p=Json::parse(text.toStdString());maps_page_->picked(p);if(p.at("frame")!=frame_->text().toStdString()||points_->rowCount()>=200)return;
   int row=points_->rowCount();points_->insertRow(row);for(int i=0;i<3;++i)points_->setItem(row,i,new QTableWidgetItem(QString::number(p.at(i==0?"x":i==1?"y":"yaw").get<double>(),'g',12)));points_->selectRow(row);
 }catch(...){}},Qt::QueuedConnection);
 connect(this,&WorkstationPanel::statusReceived,this,[this](QString text){try{if(text.size()>1024*1024)throw std::runtime_error("oversized status");auto s=Json::parse(text.toStdString());if(s.at("schema_version")!=1)throw std::runtime_error("schema");
   auto boot=s.at("boot_id").get<std::string>();if(!boot_.empty()&&boot!=boot_){lease_.clear();last_command_.clear();log_->appendPlainText("后端重启：控制权失效，旧任务需核对，禁止自动恢复");}
   if(s.contains("mapping_runtime")&&s.at("mapping_runtime").value("quality","")=="VALID") {
     auto session=s.at("mapping_runtime").at("value").value("session_id","");
     if(!mapping_session_.empty()&&session!=mapping_session_){points_->setRowCount(0);preview();log_->appendPlainText("建图会话变化：旧路线草稿已清空，请重新选点");}
     mapping_session_=session;
   }
   if(s.contains("map_catalog")&&s.at("map_catalog").value("quality","")=="VALID"){
     const auto & active=s.at("map_catalog").at("value").at("active_map");
     const auto version=active.is_null()?"":active.at("version").get<std::string>();
     if(version!=active_map_version_){points_->setRowCount(0);preview();active_map_version_=version;log_->appendPlainText("活动地图版本变化：旧路线草稿失效，请重新选点");}
   }
   boot_=boot;state_=std::move(s);seen_=Clock::now();refresh();}catch(...){seen_={};}},Qt::QueuedConnection);
 connect(this,&WorkstationPanel::responseReceived,this,[this](QString text){try{auto r=Json::parse(text.toStdString());
   if(r.at("request_operation")=="renew"){
     if(!renew_pending_||r.at("request_id")!=renew_command_)return;
     renew_pending_=false;
     if(r.at("boot_id")==boot_&&!r.at("accepted").get<bool>())lease_.clear();
     return;
   }
   if(!pending_||r.at("request_id")!=pending_command_)return;
   pending_=false;
   if(r.at("boot_id")!=boot_)return;
   const auto op=r.at("request_operation").get<std::string>();
   if(op=="acquire"&&r.at("accepted").get<bool>())lease_=Json::parse(r.at("result_json").get<std::string>()).at("lease_id");
   if(op=="release"||r.at("reason_code")=="CONTROL.NOT_OWNER")lease_.clear();
   if(op!="renew")log_->appendPlainText(QString::fromStdString(r.dump(2)));
   if(op=="renew"&&!r.at("accepted").get<bool>())lease_.clear();
   refresh();
 }catch(const std::exception & e){log_->appendPlainText(e.what());}},Qt::QueuedConnection);
 timer_=new QTimer(this);timer_->setInterval(200);connect(timer_,&QTimer::timeout,this,[this]{
   if(pending_&&Clock::now()>deadline_){command_->remove_pending_request(pending_id_);pending_=false;log_->appendPlainText("请求响应超时：结果未知，请查询原命令；不自动重新执行");}
   if(renew_pending_&&Clock::now()>renew_deadline_){command_->remove_pending_request(renew_id_);renew_pending_=false;lease_.clear();log_->appendPlainText("控制权续租未确认，已退回观察模式");}
   if(!lease_.empty()&&!renew_pending_&&Clock::now()>renew_at_&&Clock::now()-seen_<std::chrono::seconds(3)){renew_at_=Clock::now()+std::chrono::seconds(1);submit("renew",Json::object(),false);}
   refresh();});timer_->start();executor_.add_node(node_);thread_=std::thread([this]{executor_.spin();});
}
WorkstationPanel::~WorkstationPanel(){timer_->stop();executor_.cancel();if(thread_.joinable())thread_.join();}
void WorkstationPanel::refresh() {
 const bool fresh=!state_.empty()&&Clock::now()-seen_<std::chrono::seconds(3)&&status_sub_->get_publisher_count()==1;
 for(auto & [op,b]:buttons_)b->setEnabled(fresh&&!pending_&&command_->service_is_ready()&&((!lease_.empty()&&state_.value("control_state","")=="HELD")||op=="acquire"||op=="query"));
 maps_page_->updateCatalog(fresh?state_.value("map_catalog",Json::object()):Json::object(),fresh&&!pending_&&!lease_.empty()&&state_.value("control_state","")=="HELD");
 const auto sim=state_.value("simulation_transport",Json::object());
 const bool sim_valid=fresh&&sim.value("quality","")=="VALID";
 const auto sim_state=sim_valid?sim.at("value").value("state",""):"";
 buttons_.at("simulation_transport_start")->setEnabled(buttons_.at("simulation_transport_start")->isEnabled()&&(sim_state=="IDLE"||sim_state=="SUCCEEDED"));
 buttons_.at("simulation_transport_cancel")->setEnabled(buttons_.at("simulation_transport_cancel")->isEnabled()&&(sim_state=="STARTING"||sim_state=="RUNNING"));
 const auto arm_state=state_.value("arm_preview",Json::object());
 buttons_.at("arm_plan")->setEnabled(buttons_.at("arm_plan")->isEnabled()&&arm_state.value("planning_available",false)&&arm_state.value("state","")!="PLANNING");
 buttons_.at("arm_discard")->setEnabled(buttons_.at("arm_discard")->isEnabled()&&(arm_state.value("state","")=="PLANNING"||arm_state.value("state","")=="READY"));
 arm_status_->setText(fresh?QString::fromStdString(arm_state.dump(2)+"\n仿真任务："+sim.dump(2)+"\n抓放反馈："+state_.value("transport",Json::object()).dump(2)):"状态过期，预览与反馈不可作为当前状态");
 if(!fresh){summary_->setText("机器人状态未连接/过期/冲突；禁止提交新控制。软件取消不是急停。");return;}
 buttons_.at("acquire")->setEnabled(!pending_&&lease_.empty()&&state_.value("control_state","")=="OBSERVER");
 buttons_.at("query")->setEnabled(!pending_&&!last_command_.empty());
 const auto route_observation=state_.value("route",Json::object());
 const bool editing=!pending_&&!state_.value("nav_outstanding",false)&&!(route_observation.value("quality","")=="VALID"&&route_observation.at("value").value("active",false));
 for(auto b:draft_buttons_)b->setEnabled(editing);
 dwell_->setEnabled(editing);points_->setEditTriggers(editing?QAbstractItemView::DoubleClicked|QAbstractItemView::EditKeyPressed:QAbstractItemView::NoEditTriggers);
 const auto caps=state_.value("capabilities",Json::object());
 for(auto op:{"navigate","start_route","new_mapping_session"})buttons_.at(op)->setEnabled(buttons_.at(op)->isEnabled()&&caps.value(op==std::string("start_route")?"loop_route":op,false));
 if(sim.contains("value")&&!sim.at("value").is_null()&&(!sim_valid||sim.at("value").value("motion_blocked",true)))
   for(auto op:{"navigate","start_route","new_mapping_session","arm_plan","explore_resume"})buttons_.at(op)->setEnabled(false);
 auto e=state_.value("exploration",Json::object());auto m=state_.value("mapping",Json::object());
 for(auto op:{"explore_pause","explore_resume","explore_cancel_save"}){
   bool permitted=e.value("quality","")=="VALID"&&e.at("value").value("can_"+std::string(op).substr(8),false);
   if(std::string(op)!="explore_pause")permitted=permitted&&m.value("quality","")=="VALID"&&m.at("value").value("state","")=="IDLE";
   buttons_.at(op)->setEnabled(buttons_.at(op)->isEnabled()&&permitted);
 }
 buttons_.at("retry_map")->setEnabled(buttons_.at("retry_map")->isEnabled()&&m.value("quality","")=="VALID"&&m.at("value").value("state","")=="FAILED");
 QString overview=QString("机器人 %1 · %2 · 控制者 %3\n").arg(QString::fromStdString(state_.value("robot_id","")),QString::fromStdString(state_.value("control_state","")),QString::fromStdString(state_.value("control_owner","")));
 if(state_.contains("navigation"))overview+="导航："+QString::fromStdString(state_.at("navigation").value("state","UNKNOWN"))+" · "+QString::fromStdString(state_.at("navigation").value("reason_code",""))+"\n";
 const auto battery=state_.value("battery",Json::object());
 overview+="电量："+(battery.value("quality","")=="VALID"&&battery.value("value",Json()).is_number()?QString::number(battery.at("value").get<double>()*100,'f',1)+"%":"未知")+"\n";
 for(const auto & item:std::vector<std::pair<std::string,QString>>{{"mapping_runtime","建图会话"},{"exploration","探索"},{"mapping","地图保存"},{"route","循环路线"},{"recording","诊断记录"}}) {
   const auto observed=state_.value(item.first,Json::object());overview+=item.second+"：";
   if(observed.value("quality","")!="VALID"){overview+="未连接/过期\n";continue;}
   const auto value=observed.at("value");
   overview+=item.first=="recording"?(value.value("recording",false)?"正在记录":"未在记录"):QString::fromStdString(value.value("state",value.value("status","UNKNOWN")));
   for(const auto * key:{"reason_code","reason","progress","readiness_detail","transition_reason","detail","directory","fault","output_dir","session_dir","exploration_outcome"})
     if(value.contains(key)&&value.at(key).is_string()&&!value.at(key).get<std::string>().empty())overview+=" · "+QString::fromStdString(value.at(key).get<std::string>());
   if(item.first=="route")overview+=QString(" · 第 %1 点 · 完成 %2 圈").arg(value.value("index",0)+1).arg(value.value("completed_cycles",uint64_t(0)));
   if(value.value("cancel_pending",false))overview+=" · 等待导航终态，禁止恢复";
   if(value.value("session_ending",false))overview+=" · 会话已收尾，不能恢复旧会话";
   overview+="\n";
 }
 summary_->setText(overview.left(2500));
}
void WorkstationPanel::submit(const std::string & operation,Json payload,bool confirmation) {
 if((pending_&&operation!="renew")||boot_.empty()||Clock::now()-seen_>=std::chrono::seconds(3)||status_sub_->get_publisher_count()!=1)return;
 try {
   if(operation=="navigate"||operation=="start_route") {
     auto point=[&](int row){if(row<0||row>=points_->rowCount())throw std::runtime_error("请先选点");Json p={{"frame",frame_->text().toStdString()}};
       for(int i=0;i<3;++i){bool valid=false;double v=points_->item(row,i)?points_->item(row,i)->text().toDouble(&valid):0;if(!valid||!std::isfinite(v))throw std::runtime_error("坐标非法");p[i==0?"x":i==1?"y":"yaw"]=v;}return p;};
     if(operation=="navigate")payload=point(points_->currentRow());else{payload["points"]=Json::array();for(int row=0;row<points_->rowCount();++row)payload["points"].push_back(point(row));payload["dwell_sec"]=dwell_->value();}
   }
   if(operation=="arm_plan")payload={{"group",arm_group_->currentText().toStdString()},{"named_target","transport_compact"}};
   if(operation=="arm_discard")payload["plan_id"]=state_.at("arm_preview").at("plan_id");
   if(operation=="acquire")payload["client_name"]=node_->get_name();
   if(operation=="cancel_navigation")payload["operation_id"]=state_.at("nav_operation_id");
   if(operation=="cancel_route")payload["route_id"]=state_.at("route_id");
   if(operation.rfind("explore_",0)==0){const auto & e=state_.at("exploration").at("value");payload["expected_exploration_boot"]=e.at("boot_id");payload["expected_exploration_revision"]=e.at("revision");}
   if(operation=="navigate"||operation=="start_route"||operation=="explore_resume"||operation=="new_mapping_session")payload["map_version"]=active_map_version_;
   const auto boot=boot_;const auto lease=lease_;
   if(confirmation&&operation!="query"&&operation!="acquire"&&operation!="explore_pause"&&operation!="cancel_navigation"&&operation!="cancel_route") {
     QString description=QString::fromStdString(operation);
     if(operation=="simulation_transport_start")description="在当前仿真场景执行取货、送达和放置";
     if(operation=="simulation_transport_cancel")description="请求取消仿真搬运，保留载荷并等待停止确认";
     if(operation=="navigate")description="单点导航（operator 优先级，可能抢占路线/探索）\n"+QString::fromStdString(payload.dump());
     if(operation=="start_route")description=QString("按 %1 个目标点无限循环，每点等待 %2 秒").arg(payload.at("points").size()).arg(dwell_->value());
     QMessageBox box(QMessageBox::Question,"确认操作",description+(operation=="explore_cancel_save"?"\n结束后不能恢复旧 SLAM 会话。继续？":"\n以任务状态反馈为准；取消受理不代表停稳。继续？"),QMessageBox::Yes|QMessageBox::No,this);
     box.setDetailedText(QString::fromStdString(payload.dump(2)));if(box.exec()!=QMessageBox::Yes)return;
     if(pending_||boot!=boot_||lease!=lease_||Clock::now()-seen_>=std::chrono::seconds(3))return;
   }
   if(operation.rfind("explore_",0)==0) {
     const auto & e=state_.at("exploration").at("value");
     if(payload.at("expected_exploration_boot")!=e.at("boot_id")||payload.at("expected_exploration_revision")!=e.at("revision")){log_->appendPlainText("探索状态在确认期间变化，请重新核对");return;}
   }
   auto q=std::make_shared<Command::Request>();q->robot_id=state_.at("robot_id");q->expected_boot_id=boot_;q->lease_id=lease_;q->operation=operation;q->payload_json=payload.dump();
   q->command_id=operation=="query"?last_command_:QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
   if(operation!="renew"&&operation!="release"&&operation!="acquire"&&operation!="query")last_command_=q->command_id;
   if(operation=="renew"){renew_command_=q->command_id;renew_pending_=true;renew_deadline_=Clock::now()+std::chrono::seconds(3);}
   else{pending_command_=q->command_id;pending_=true;deadline_=Clock::now()+std::chrono::seconds(3);}
   auto f=command_->async_send_request(q,[this,operation,id=q->command_id](rclcpp::Client<Command>::SharedFuture f){try{auto r=f.get();emit responseReceived(QString::fromStdString(Json({{"request_id",id},{"request_operation",operation},{"accepted",r->accepted},{"reason_code",r->reason_code},{"message",r->message},{"boot_id",r->boot_id},{"state",r->state},{"operation_id",r->operation_id},{"result_json",r->result_json}}).dump()));}catch(...){} });if(operation=="renew")renew_id_=f.request_id;else pending_id_=f.request_id;
 }catch(const std::exception & e){if(operation=="renew")renew_pending_=false;else pending_=false;log_->appendPlainText(e.what());}
 refresh();
}
void WorkstationPanel::preview() {
 if(!preview_)return;
 using Marker=visualization_msgs::msg::Marker;visualization_msgs::msg::MarkerArray out;
 for(int id=0;id<marker_count_;++id){Marker m;m.ns=node_->get_name();m.id=id;m.action=Marker::DELETE;out.markers.push_back(m);}marker_count_=0;
 for(int row=0;row<points_->rowCount();++row){double v[3];bool valid=true;
   for(int c=0;c<3;++c){bool ok=false;v[c]=points_->item(row,c)?points_->item(row,c)->text().toDouble(&ok):0;valid=valid&&ok&&std::isfinite(v[c]);}if(!valid)continue;
   Marker m;m.header.frame_id=frame_->text().toStdString();m.ns=node_->get_name();m.id=marker_count_++;m.type=Marker::ARROW;m.action=Marker::ADD;
   m.pose.position.x=v[0];m.pose.position.y=v[1];m.pose.orientation.z=std::sin(v[2]/2);m.pose.orientation.w=std::cos(v[2]/2);m.scale.x=.5;m.scale.y=.08;m.scale.z=.08;m.color.a=1;m.color.g=1;m.color.b=1;out.markers.push_back(m);
   m.id=marker_count_++;m.type=Marker::TEXT_VIEW_FACING;m.pose.position.z=.4;m.scale.z=.25;m.text=std::to_string(row+1);out.markers.push_back(m);
 }
 frame_->setReadOnly(points_->rowCount()>0);preview_->publish(out);
}
void WorkstationPanel::routeFile(bool save) {
 const auto path=save?QFileDialog::getSaveFileName(this,"保存路线",{},"JSON (*.json)"):QFileDialog::getOpenFileName(this,"加载路线",{},"JSON (*.json)");if(path.isEmpty())return;
 try {
 if(save){Json rows=Json::array();for(int row=0;row<points_->rowCount();++row){Json p=Json::array();for(int c=0;c<3;++c){bool ok=false;double v=points_->item(row,c)?points_->item(row,c)->text().toDouble(&ok):0;if(!ok||!std::isfinite(v))throw std::runtime_error("坐标非法");p.push_back(v);}rows.push_back(p);}
   Json j={{"version",1},{"frame",frame_->text().toStdString()},{"dwell_sec",dwell_->value()},{"points_xy_yaw_rad",rows}};QSaveFile file(path);if(!file.open(QIODevice::WriteOnly)||file.write(QByteArray::fromStdString(j.dump(2)))<0||!file.commit())throw std::runtime_error("保存失败");
 }else{QFile file(path);if(!file.open(QIODevice::ReadOnly)||file.size()>1024*1024)throw std::runtime_error("文件不可读或过大");auto j=Json::parse(file.readAll().toStdString());auto rows=j.at("points_xy_yaw_rad");auto frame=j.at("frame").get<std::string>();double dwell=j.at("dwell_sec");
   if(j.at("version")!=1||!rows.is_array()||rows.size()>200||frame.empty()||frame.front()=='/'||!std::isfinite(dwell)||dwell<0||dwell>3600)throw std::runtime_error("路线格式非法");
   for(const auto & row:rows){if(!row.is_array()||row.size()!=3)throw std::runtime_error("坐标格式非法");for(const auto & v:row)if(!v.is_number()||!std::isfinite(v.get<double>()))throw std::runtime_error("坐标非法");}
   points_->blockSignals(true);points_->setRowCount(0);frame_->setText(QString::fromStdString(frame));dwell_->setValue(dwell);
   for(const auto & row:rows){int n=points_->rowCount();points_->insertRow(n);for(int c=0;c<3;++c)points_->setItem(n,c,new QTableWidgetItem(QString::number(row[c].get<double>(),'g',15)));}points_->blockSignals(false);preview();
 }
 }catch(const std::exception & e){log_->appendPlainText(e.what());}
}
void WorkstationPanel::replay(){auto directory=QFileDialog::getExistingDirectory(this,"选择诊断包目录（含 bag 与 events.jsonl）");if(directory.isEmpty())return;
 if(!QProcess::startDetached(QString::fromStdString(ament_index_cpp::get_package_prefix("astribot_operator_station"))+"/lib/astribot_operator_station/replay_viewer",{directory}))log_->appendPlainText("回放进程启动失败");}
}
PLUGINLIB_EXPORT_CLASS(astribot_operator_station::WorkstationPanel,rviz_common::Panel)
