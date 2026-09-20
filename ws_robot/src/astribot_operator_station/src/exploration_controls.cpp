#include "astribot_operator_station/exploration_controls.hpp"
#include <QVBoxLayout>
#include <QMessageBox>
#include <QUuid>
namespace astribot_operator_station {
using Json=nlohmann::json;
ExplorationControls::ExplorationControls(rclcpp::Node::SharedPtr node,std::function<bool()> enabled,QWidget * parent)
 :QWidget(parent),node_(node),enabled_(std::move(enabled)) {
 auto layout=new QVBoxLayout(this);
 summary_=new QLabel("探索状态未连接；不提供新 SLAM 会话启动");summary_->setObjectName("exploration_summary");summary_->setWordWrap(true);layout->addWidget(summary_);
 auto button=[&](const char * text){auto p=new QPushButton(text);p->setEnabled(false);layout->addWidget(p);return p;};
 pause_=button("暂停探索");resume_=button("恢复当前探索");cancel_=button("取消探索并保存地图");
 save_retry_=button("重试地图保存");resolve_=button("重试确认同一探索请求");
 client_=node_->create_client<Command>("/exploration_coordinator_node/command");
 retry_=node_->create_client<std_srvs::srv::Trigger>("/mapping_session/retry");
 connect(pause_,&QPushButton::clicked,this,[this]{command("pause");});
 connect(resume_,&QPushButton::clicked,this,[this]{command("resume");});
 connect(cancel_,&QPushButton::clicked,this,[this]{command("cancel_save");});
 connect(resolve_,&QPushButton::clicked,this,[this]{if(uncertain_&&request_&&enabled_()&&client_->service_is_ready())sendRequest();});
 connect(save_retry_,&QPushButton::clicked,this,[this]{
   if(!save_retry_->isEnabled())return;
   retry_pending_=true;retry_deadline_=Clock::now()+std::chrono::seconds(5);refresh();
   try {auto future=retry_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f){
     try {auto r=f.get();emit retryReply(r->success,QString::fromStdString(r->message));}
     catch(const std::exception & e){emit retryReply(false,e.what());}
   });retry_id_=future.request_id;}catch(const std::exception & e){retry_pending_=false;emit report(e.what());}
 });
 connect(this,&ExplorationControls::retryReply,this,[this](bool accepted,QString message){
   if(!retry_pending_)return;
   retry_pending_=false;
   emit report((accepted?"存图重试已受理，等待 SAVED：":"存图重试未受理：")+message);refresh();
 },Qt::QueuedConnection);
 connect(this,&ExplorationControls::statusReceived,this,[this](QString source,QString data){
   try {
     const auto value=Json::parse(data.toStdString());
     if(source=="exploration") {
       if(value.at("schema_version")!=1||value.at("boot_id").get<std::string>().empty())throw std::runtime_error("不支持的探索状态协议");
       for(const auto * key:{"can_pause","can_resume","can_cancel_save","session_ending","cancel_pending","manual_pause"})value.at(key).get<bool>();
       value.at("revision").get<uint64_t>();value.at("reason_code").get<std::string>();value.at("progress").get<std::string>();
       for(const auto * key:{"readiness_detail","transition_reason"})if(value.contains(key))value.at(key).get<std::string>();
       if(!exploration_.empty()&&exploration_.at("boot_id")==value.at("boot_id")&&value.at("revision").get<uint64_t>()<exploration_.at("revision").get<uint64_t>())return;
       if(!exploration_.empty()&&exploration_.at("boot_id")!=value.at("boot_id")) {
         if(pending_)client_->remove_pending_request(request_id_);
         pending_=false;uncertain_=false;request_.reset();
         emit report("探索实例已变化，旧请求作废；请重新核对会话，不能从空闲推断停稳");
       }
       exploration_=value;exploration_at_=Clock::now();
     } else {
       value.at("state").get<std::string>();value.at("directory").get<std::string>();
       for(const auto * key:{"detail","exploration_outcome"})if(value.contains(key))value.at(key).get<std::string>();
       mapping_=value;mapping_at_=Clock::now();
     }
   }catch(const std::exception & e){if(source=="exploration")exploration_at_={};else mapping_at_={};emit report(QString("状态协议错误：")+e.what());}
   refresh();
 },Qt::QueuedConnection);
 connect(this,&ExplorationControls::commandReply,this,[this](QString id,QString boot,bool accepted,QString code,QString message){
   if(!request_||id.toStdString()!=request_->command_id||boot.toStdString()!=request_->expected_boot_id)return;
   pending_=false;uncertain_=false;request_.reset();
   emit report((accepted?"探索命令已受理（不代表动作完成）：":"探索命令被拒绝：")+code+" · "+message);refresh();
 },Qt::QueuedConnection);
 for(const auto & item:std::vector<std::pair<std::string,std::string>>{{"exploration","/exploration_coordinator_node/operator_status"},{"mapping","/mapping_session/status"}}) {
   subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(item.second,rclcpp::QoS(1).transient_local(),
    [this,source=item.first](std_msgs::msg::String::ConstSharedPtr m){emit statusReceived(QString::fromStdString(source),QString::fromStdString(m->data));}));
 }
 timer_=new QTimer(this);timer_->setInterval(100);connect(timer_,&QTimer::timeout,this,[this]{refresh();});timer_->start();
}
ExplorationControls::~ExplorationControls(){timer_->stop();}
bool ExplorationControls::allowed(const std::string & operation) const {
 if(subscriptions_.at(0)->get_publisher_count()!=1)return false;
 if(!enabled_()||pending_||uncertain_||exploration_.empty()||Clock::now()-exploration_at_>=std::chrono::seconds(3)||!client_->service_is_ready())return false;
 if(operation!="pause"&&(subscriptions_.at(1)->get_publisher_count()!=1||mapping_.empty()||Clock::now()-mapping_at_>=std::chrono::seconds(3)||mapping_.at("state")!="IDLE"))return false;
 return exploration_.value("can_"+operation,false);
}
void ExplorationControls::refresh() {
 const auto now=Clock::now();
 if(pending_&&now>=deadline_){client_->remove_pending_request(request_id_);pending_=false;uncertain_=true;emit report("探索响应超时，结果未知；仅允许重试确认原命令，不能重建命令");}
 if(retry_pending_&&now>=retry_deadline_){retry_->remove_pending_request(retry_id_);retry_pending_=false;mapping_at_={};emit report("存图重试响应超时，先等待新的地图状态，不能认为已保存");}
 pause_->setEnabled(allowed("pause"));resume_->setEnabled(allowed("resume"));cancel_->setEnabled(allowed("cancel_save"));
 const bool map_fresh=subscriptions_.at(1)->get_publisher_count()==1&&!mapping_.empty()&&now-mapping_at_<std::chrono::seconds(3);
 save_retry_->setEnabled(enabled_()&&map_fresh&&mapping_.at("state")=="FAILED"&&!retry_pending_&&retry_->service_is_ready());
 resolve_->setEnabled(enabled_()&&uncertain_&&!pending_&&request_&&client_->service_is_ready()&&now-exploration_at_<std::chrono::seconds(3));
 QString text;
 if(subscriptions_.at(0)->get_publisher_count()!=1||exploration_.empty()||now-exploration_at_>=std::chrono::seconds(3))text="探索状态未连接/过期：禁止提交新探索命令";
 else if(exploration_.value("cancel_pending",false))text="等待导航取消终态；未确认停稳，禁止恢复";
 else if(exploration_.value("session_ending",false))text="探索会话已结束，正在等待建图收尾；不能恢复旧会话";
 else if(exploration_.value("reason_code","")=="EXPLORATION.NOT_READY")text="等待地图/定位/里程计/校验图就绪；不是建图完成";
 else if(exploration_.value("manual_pause",false))text="探索已人工暂停/被接管；核对原因后恢复，不自动抢回控制";
 else if(exploration_.value("reason_code","")=="EXPLORATION.RETRY_WAIT")text="探索失败冷却或重试预算已耗尽；核对原因和就绪条件";
 else text="探索运行中";
 if(subscriptions_.at(0)->get_publisher_count()==1&&!exploration_.empty()&&now-exploration_at_<std::chrono::seconds(3)) {
   const auto progress=exploration_.value("progress","");
   static const std::map<std::string,QString> hints={{"UNREACHABLE_FRONTIERS","有边界但不可达，不能视为完成"},{"UNOBSERVABLE_UNKNOWN","仍有无法观测的未知区域"},{"CANDIDATE_COOLDOWN","候选点冷却等待"},{"CONFIRMING_COMPLETE","无目标，正在确认完成条件"},{"CURRENT_MAP_COMPLETE","后端确认当前地图探索完成；不代表全部楼层覆盖"},{"INSUFFICIENT_KNOWN_MAP","已知区域不足，等待地图"},{"MAP_CHANGED_RECHECK","地图变化，重新检查"}};
   if(hints.count(progress))text+="\n"+hints.at(progress);
   text+="\n"+QString::fromStdString(exploration_.value("readiness_detail","")+" "+exploration_.value("transition_reason",""));
 }
 if(!map_fresh)text+="\n地图保存状态未连接/过期，禁止恢复、结束保存与保存重试";
 else {
   const auto state=mapping_.at("state").get<std::string>();
   if(state=="SAVED")text+="\n地图已保存（"+QString::fromStdString(mapping_.value("exploration_outcome","UNKNOWN"))+"）；新建图需新会话";
   else if(state=="FAILED")text+="\n地图保存失败，可核对原因后重试保存；不能恢复探索";
   else if(state!="IDLE")text+="\n地图收尾阶段："+QString::fromStdString(state)+"；等待保存结果";
   text+="\n"+QString::fromStdString(mapping_.value("detail","")+" "+mapping_.value("directory",""));
 }
 summary_->setText(text);
}
void ExplorationControls::command(const std::string & operation) {
 if(!allowed(operation))return;
 const auto boot=exploration_.at("boot_id").get<std::string>();const auto revision=exploration_.at("revision").get<uint64_t>();
 if(operation!="pause") {
   const auto message=operation=="cancel_save"?"结束当前探索并保存部分地图，不能再恢复本 SLAM 会话。继续？":"恢复当前探索并重置失败预算，可能重新产生导航目标。继续？";
   if(QMessageBox::question(this,"探索操作确认",message)!=QMessageBox::Yes)return;
   if(!allowed(operation)||exploration_.at("boot_id")!=boot||exploration_.at("revision")!=revision){emit report("确认期间探索状态变化或过期，请重新核对");return;}
 }
 request_=std::make_shared<Command::Request>();request_->command_id=QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
 request_->expected_boot_id=boot;request_->expected_revision=revision;request_->operation=operation;sendRequest();
}
void ExplorationControls::sendRequest() {
 pending_=true;uncertain_=false;deadline_=Clock::now()+std::chrono::seconds(5);refresh();
 try {auto f=client_->async_send_request(request_,[this,id=request_->command_id](rclcpp::Client<Command>::SharedFuture future){
   try {auto r=future.get();emit commandReply(QString::fromStdString(id),QString::fromStdString(r->boot_id),r->accepted,QString::fromStdString(r->reason_code),QString::fromStdString(r->message));}
   catch(const std::exception & e){emit report(e.what());}
 });request_id_=f.request_id;}catch(const std::exception & e){pending_=false;uncertain_=true;emit report(e.what());}
}
}
