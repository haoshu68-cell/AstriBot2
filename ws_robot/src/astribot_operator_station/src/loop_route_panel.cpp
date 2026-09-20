#include "astribot_operator_station/loop_route_panel.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nlohmann/json.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QHeaderView>
#include <QMessageBox>
#include <QFileDialog>
#include <QSaveFile>
#include <QUuid>
#include <cmath>
#include <atomic>
#include <unistd.h>
namespace astribot_operator_station {
using Json=nlohmann::json;
static constexpr double degrees=180./3.14159265358979323846;
LoopRoutePanel::LoopRoutePanel(QWidget * parent):rviz_common::Panel(parent) {
 auto layout=new QVBoxLayout(this);
 auto help=new QLabel("点击“地图连续选点”，在地图上按下并拖动设置朝向，松开添加。\n按列表顺序 1→2→…→1 循环；失败/抢占停止。关闭面板不取消路线。");
 help->setWordWrap(true);layout->addWidget(help);
 auto form=new QFormLayout;frame_=new QLineEdit("map");frame_->setObjectName("route_frame");
 dwell_=new QDoubleSpinBox;dwell_->setRange(0,3600);dwell_->setValue(0.5);dwell_->setSuffix(" s");
 form->addRow("坐标系（须与选点一致）",frame_);form->addRow("每点成功后的等待",dwell_);layout->addLayout(form);
 picking_=new QCheckBox("接收地图选点");picking_->setChecked(true);layout->addWidget(picking_);
 table_=new QTableWidget(0,3);table_->setObjectName("route_points");table_->setHorizontalHeaderLabels({"X (m)","Y (m)","Yaw (°)"});
 table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);table_->setSelectionBehavior(QAbstractItemView::SelectRows);
 table_->setSelectionMode(QAbstractItemView::SingleSelection);layout->addWidget(table_);
 auto row=new QHBoxLayout;layout->addLayout(row);
 auto button=[&](const QString & label,auto function){auto b=new QPushButton(label);row->addWidget(b);editors_.push_back(b);connect(b,&QPushButton::clicked,this,function);return b;};
 button("地图连续选点",[this]{
   if(!getDisplayContext())return;
   auto manager=getDisplayContext()->getToolManager();
   for(int i=0;i<manager->numTools();++i)if(manager->getTool(i)->getClassId()=="astribot_operator_station/RoutePointTool") {
     picking_->setChecked(true);manager->setCurrentTool(manager->getTool(i));return;
   }
   emit report("请在 RViz Tools 中添加 astribot_operator_station/RoutePointTool");
 });
 button("添加坐标行",[this]{addPoint(0,0,0);});
 button("删除",[this]{table_->removeRow(table_->currentRow());preview();});
 button("上移",[this]{reorder(-1);});button("下移",[this]{reorder(1);});
 row=new QHBoxLayout;layout->addLayout(row);
 button("清空",[this]{table_->setRowCount(0);preview();});
 button("保存路线",[this]{routeFile(true);});button("加载路线",[this]{routeFile(false);});
 enable_=new QCheckBox("允许启动循环导航");layout->addWidget(enable_);
 start_button_=new QPushButton("确认并开始循环");cancel_button_=new QPushButton("取消当前循环路线");
 layout->addWidget(start_button_);layout->addWidget(cancel_button_);
 status_=new QLabel("等待路线执行器状态；尚未下发任务");status_->setWordWrap(true);layout->addWidget(status_);
 output_=new QPlainTextEdit;output_->setReadOnly(true);output_->setMaximumBlockCount(100);layout->addWidget(output_);
 editors_.insert(editors_.end(),{table_,frame_,dwell_,picking_});
 static std::atomic<unsigned> instance{0};
 node_=std::make_shared<rclcpp::Node>("loop_route_panel_"+std::to_string(getpid())+"_"+std::to_string(instance++));
 start_client_=node_->create_client<Start>("/loop_route_executor/start");
 cancel_client_=node_->create_client<Cancel>("/loop_route_executor/cancel");
 markers_=node_->create_publisher<visualization_msgs::msg::MarkerArray>("/operator/route_preview",rclcpp::QoS(1).transient_local());
 status_sub_=node_->create_subscription<std_msgs::msg::String>("/loop_route_executor/status",rclcpp::QoS(1).transient_local(),
  [this](std_msgs::msg::String::ConstSharedPtr m){emit receivedStatus(QString::fromStdString(m->data));});
 point_sub_=node_->create_subscription<geometry_msgs::msg::PoseStamped>("/operator/route_point",10,
  [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr p){const auto & q=p->pose.orientation;
   emit receivedPoint(QString::fromStdString(Json({{"frame",p->header.frame_id},{"x",p->pose.position.x},{"y",p->pose.position.y},{"yaw",std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}}).dump()));});
 connect(this,&LoopRoutePanel::report,output_,&QPlainTextEdit::appendPlainText,Qt::QueuedConnection);
 connect(this,&LoopRoutePanel::receivedStatus,this,&LoopRoutePanel::updateStatus,Qt::QueuedConnection);
 connect(this,&LoopRoutePanel::receivedPoint,this,[this](QString data){
   if(active_||pending_||uncertain_||!picking_->isChecked())return;
   try {auto p=Json::parse(data.toStdString());if(p["frame"]!=frame_->text().trimmed().toStdString()) {emit report("选点坐标系与路线不一致，请统一 RViz Fixed Frame 和路线坐标系");return;}
     addPoint(p.at("x"),p.at("y"),p.at("yaw"));}catch(const std::exception & e){emit report(e.what());}
 },Qt::QueuedConnection);
 connect(this,&LoopRoutePanel::startReply,this,[this](QString data){
   auto response=Json::parse(data.toStdString());
   if(!pending_request_ || response.at("request_id")!=pending_request_->request_id || response.at("boot_id")!=boot_id_) {
     emit report("忽略旧请求或旧执行器的迟到启动响应");return;
   }
   pending_=false;uncertain_=false;
   if(response["accepted"].get<bool>()) {active_=response.at("active").get<bool>();route_id_=response["route_id"].get<std::string>();}
   emit report(QString::fromStdString(response["reason"].get<std::string>()));
 },Qt::QueuedConnection);
 connect(this,&LoopRoutePanel::cancelReply,this,[this](QString text){cancel_pending_=false;emit report(text);},Qt::QueuedConnection);
 connect(start_button_,&QPushButton::clicked,this,&LoopRoutePanel::startRoute);
 connect(cancel_button_,&QPushButton::clicked,this,&LoopRoutePanel::cancelRoute);
 connect(table_,&QTableWidget::cellChanged,this,[this]{if(!updating_)preview();});
 connect(frame_,&QLineEdit::editingFinished,this,&LoopRoutePanel::preview);
 timer_=new QTimer(this);timer_->setInterval(100);
 connect(timer_,&QTimer::timeout,this,[this]{
   const bool fresh=std::chrono::steady_clock::now()-seen_<std::chrono::seconds(3);
   if(pending_ && std::chrono::steady_clock::now()>deadline_) {
     start_client_->remove_pending_request(request_id_);pending_=false;uncertain_=true;
     emit report("启动响应超时，结果未知；可用相同请求重试确认，勿重新创建任务");
   }
   if(cancel_pending_ && std::chrono::steady_clock::now()>cancel_deadline_) {
     cancel_client_->remove_pending_request(cancel_request_id_);cancel_pending_=false;
     emit report("取消请求响应超时；未确认停止，可重试取消同一路线并核对状态");
   }
   bool editing=!active_&&!pending_&&!uncertain_;
   for(auto widget:editors_)widget->setEnabled(editing);
   frame_->setEnabled(editing&&table_->rowCount()==0);
   table_->setEnabled(true);
   table_->setEditTriggers(editing?(QAbstractItemView::DoubleClicked|QAbstractItemView::EditKeyPressed):QAbstractItemView::NoEditTriggers);
   start_button_->setText(uncertain_?"重试确认同一启动请求":"确认并开始循环");
   start_button_->setEnabled(enable_->isChecked()&&!pending_&&start_client_->service_is_ready()&&fresh&&((!active_&&table_->rowCount()>=2)||uncertain_));
   cancel_button_->setEnabled(!cancel_pending_&&active_&&!route_id_.empty()&&cancel_client_->service_is_ready());
   if(!fresh)status_->setText("执行器状态过期/未连接；禁止启动。已受理路线可能仍在运行，可请求取消。");
 });timer_->start();executor_.add_node(node_);thread_=std::thread([this]{executor_.spin();});
}
LoopRoutePanel::~LoopRoutePanel(){timer_->stop();executor_.cancel();if(thread_.joinable())thread_.join();}
void LoopRoutePanel::addPoint(double x,double y,double yaw) {
 if(table_->rowCount()>=200||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)) {emit report("点数超限或坐标非法");return;}
 const bool was_updating=updating_;updating_=true;int row=table_->rowCount();table_->insertRow(row);
 for(int column=0;column<3;++column)table_->setItem(row,column,new QTableWidgetItem(QString::number(column==0?x:column==1?y:yaw*degrees,'g',15)));
 updating_=was_updating;table_->selectRow(row);if(!updating_)preview();
}
void LoopRoutePanel::reorder(int delta) {
 int row=table_->currentRow(), target=row+delta;if(row<0||target<0||target>=table_->rowCount())return;
 updating_=true;for(int c=0;c<3;++c){auto a=table_->takeItem(row,c);auto b=table_->takeItem(target,c);table_->setItem(row,c,b);table_->setItem(target,c,a);}
 updating_=false;table_->selectRow(target);preview();
}
LoopRoutePanel::Start::Request::SharedPtr LoopRoutePanel::draft() const {
 auto req=std::make_shared<Start::Request>();req->dwell_sec=dwell_->value();
 auto frame=frame_->text().trimmed().toStdString();if(frame.empty()||frame.front()=='/')throw std::runtime_error("坐标系不能为空或以 / 开头");
 for(int r=0;r<table_->rowCount();++r){double v[3];for(int c=0;c<3;++c){bool ok=false;v[c]=table_->item(r,c)?table_->item(r,c)->text().toDouble(&ok):0;if(!ok||!std::isfinite(v[c]))throw std::runtime_error("坐标或角度不是有限数字");}
   geometry_msgs::msg::PoseStamped p;p.header.frame_id=frame;p.pose.position.x=v[0];p.pose.position.y=v[1];p.pose.orientation.z=std::sin(v[2]/degrees/2);p.pose.orientation.w=std::cos(v[2]/degrees/2);req->waypoints.push_back(p);}
 return req;
}
void LoopRoutePanel::preview() {
 if(updating_)return;
 visualization_msgs::msg::MarkerArray array;
 using Marker=visualization_msgs::msg::Marker;
 for(int id=0;id<old_markers_;++id){Marker m;m.ns=node_->get_name();m.id=id;m.action=Marker::DELETE;array.markers.push_back(m);}
 old_markers_=0;
 try {auto req=draft();for(size_t i=0;i<req->waypoints.size();++i){
   Marker m;m.header=req->waypoints[i].header;m.ns=node_->get_name();m.id=old_markers_++;m.type=Marker::ARROW;m.action=Marker::ADD;m.pose=req->waypoints[i].pose;
   m.scale.x=.5;m.scale.y=.08;m.scale.z=.08;m.color.a=1;m.color.g=.8;m.color.r=(active_&&static_cast<int>(i)==current_index_)?1:0;m.color.b=m.color.r?0:1;array.markers.push_back(m);
   m.id=old_markers_++;m.type=Marker::TEXT_VIEW_FACING;m.pose.position.z=.45;m.scale.z=.28;m.text=std::to_string(i+1);array.markers.push_back(m);
 }}catch(const std::exception & e){emit report(e.what());}
 markers_->publish(array);
}
void LoopRoutePanel::startRoute() {
 if(pending_||!enable_->isChecked()||boot_id_.empty())return;
 try {
   if(!uncertain_) {
     auto req=draft();if(req->waypoints.size()<2)return;
     const auto confirmed_boot=boot_id_;
     if(QMessageBox::question(this,"开始循环路线",QString("将按列表中的 %1 个点无限循环，走 route 仲裁入口。失败或抢占会停止；关闭 RViz 不会取消。开始？").arg(req->waypoints.size()))!=QMessageBox::Yes)return;
     if(boot_id_!=confirmed_boot || !enable_->isChecked() || active_ ||
        std::chrono::steady_clock::now()-seen_>=std::chrono::seconds(3)) {
       emit report("确认期间执行器状态发生变化或过期，请重新核对后启动。");return;
     }
     req->expected_boot_id=confirmed_boot;req->request_id=QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();pending_request_=req;
   }
   pending_=true;deadline_=std::chrono::steady_clock::now()+std::chrono::seconds(5);
   auto future=start_client_->async_send_request(pending_request_,[this,id=pending_request_->request_id](rclcpp::Client<Start>::SharedFuture f){
     try {auto r=f.get();emit startReply(QString::fromStdString(Json({{"request_id",id},{"boot_id",r->boot_id},{"active",r->active},{"accepted",r->accepted},{"route_id",r->route_id},{"reason",r->reason}}).dump()));}
     catch(const std::exception & e){emit report(e.what());}
   });request_id_=future.request_id;
 }catch(const std::exception & e){pending_=false;emit report(e.what());}
}
void LoopRoutePanel::cancelRoute() {
 if(route_id_.empty()||cancel_pending_)return;
 auto req=std::make_shared<Cancel::Request>();req->route_id=route_id_;
 cancel_pending_=true;cancel_deadline_=std::chrono::steady_clock::now()+std::chrono::seconds(5);
 auto future=cancel_client_->async_send_request(req,[this](rclcpp::Client<Cancel>::SharedFuture f){
   try{auto r=f.get();emit cancelReply(QString::fromStdString((r->accepted?"取消请求已受理：":"取消被拒绝：")+r->reason));}
   catch(const std::exception & e){emit cancelReply(e.what());}
 });cancel_request_id_=future.request_id;
}
void LoopRoutePanel::updateStatus(const QString & text) {
 try {const auto s=Json::parse(text.toStdString());
   const auto boot=s.at("boot_id").get<std::string>();if(boot.empty())throw std::runtime_error("Missing executor boot ID");
   if(!boot_id_.empty() && boot!=boot_id_) {
     if(pending_)start_client_->remove_pending_request(request_id_);
     if(cancel_pending_)cancel_client_->remove_pending_request(cancel_request_id_);
     pending_=false;cancel_pending_=false;uncertain_=false;pending_request_.reset();enable_->setChecked(false);
     emit report("执行器实例已变化，旧请求作废；空闲状态不能证明旧导航已停稳。重新核对并确认后才能启动。");
   }
   boot_id_=boot;seen_=std::chrono::steady_clock::now();
   const bool was_active=active_;const int previous_index=current_index_;current_index_=s.at("index").get<int>();
   const auto previous=route_id_;route_id_=s.at("route_id").get<std::string>();active_=s.at("active");
   static const std::map<std::string,QString> labels={{"IDLE","空闲"},{"WAIT_SERVER","等待导航服务"},{"DISPATCHING","等待目标受理"},{"NAVIGATING","逐点导航"},{"DWELL","到点等待"},{"CANCELING","正在取消"},{"CANCEL_UNCONFIRMED","取消尚未确认"},{"CANCELED","已取消"},{"FAILED","路线失败"}};
   const auto code=s.at("state").get<std::string>();
   const auto label=(labels.count(code)?labels.at(code):QString::fromStdString(code))+" ("+QString::fromStdString(code)+")";
   status_->setText(QString("路线 %1\n%2 · 第 %3 点 / %4 点 · 完成 %5 圈\n%6").arg(QString::fromStdString(route_id_),label,QString::number(s.at("index").get<int>()+1),QString::number(s.at("waypoints").size()),QString::number(s.at("completed_cycles").get<uint64_t>()),QString::fromStdString(s.at("reason"))));
   if(active_ && (previous!=route_id_ || table_->rowCount()==0)) {
     updating_=true;table_->setRowCount(0);
     for(const auto & p:s.at("waypoints")) {frame_->setText(QString::fromStdString(p.at("frame")));addPoint(p.at("x"),p.at("y"),p.at("yaw"));}
     dwell_->setValue(s.at("dwell_sec"));updating_=false;preview();
   }
   if(active_)table_->selectRow(current_index_);
   if(was_active!=active_||previous_index!=current_index_)preview();
 }catch(const std::exception & e){emit report(QString("无效路线状态: ")+e.what());}
}
void LoopRoutePanel::routeFile(bool save) {
 const auto file=save?QFileDialog::getSaveFileName(this,"保存路线",{},"JSON (*.json)"):QFileDialog::getOpenFileName(this,"加载路线",{},"JSON (*.json)");
 if(file.isEmpty())return;
 try{
   if(save){auto req=draft();Json points=Json::array();for(const auto & p:req->waypoints)points.push_back({p.pose.position.x,p.pose.position.y,2*std::atan2(p.pose.orientation.z,p.pose.orientation.w)});
     Json j={{"version",1},{"frame",frame_->text().trimmed().toStdString()},{"dwell_sec",req->dwell_sec},{"points_xy_yaw_rad",points}};
     QSaveFile out(file);if(!out.open(QIODevice::WriteOnly)||out.write(QByteArray::fromStdString(j.dump(2)))<0||!out.commit())throw std::runtime_error("保存失败");
   }else{QFile in(file);if(!in.open(QIODevice::ReadOnly)||in.size()>1024*1024)throw std::runtime_error("文件不可读或超过 1 MiB");auto j=Json::parse(in.readAll().toStdString());
     auto points=j.at("points_xy_yaw_rad");auto frame=j.at("frame").get<std::string>();double dwell=j.at("dwell_sec");
     if(j.at("version")!=1||!points.is_array()||points.size()>200||frame.empty()||frame.front()=='/'||!std::isfinite(dwell)||dwell<0||dwell>3600)throw std::runtime_error("路线格式非法");
     for(const auto & p:points){if(!p.is_array()||p.size()!=3)throw std::runtime_error("坐标格式非法");for(const auto & v:p)if(!std::isfinite(v.get<double>()))throw std::runtime_error("坐标非法");}
     table_->setRowCount(0);frame_->setText(QString::fromStdString(frame));dwell_->setValue(dwell);for(const auto & p:points)addPoint(p[0],p[1],p[2]);preview();
   }
 }catch(const std::exception & e){emit report(e.what());}
}
}
PLUGINLIB_EXPORT_CLASS(astribot_operator_station::LoopRoutePanel,rviz_common::Panel)
