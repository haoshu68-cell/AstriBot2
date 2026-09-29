#include "astribot_operator_station/zone_page.hpp"
#include "astribot_navigation_zones/geometry.hpp"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QUuid>
#include <QMessageBox>
namespace astribot_operator_station {
ZonePage::ZonePage(QWidget *parent):QWidget(parent){
 auto layout=new QVBoxLayout(this);status_=new QLabel("等待地图会话与禁区状态");status_->setWordWrap(true);layout->addWidget(status_);
 auto hint=new QLabel("约束属于地图版本。先暂停探索、取消导航并等待停稳，再绘制和保存。红色为服务端约束，黄色为未保存草稿；保存成功后等待各模块生效。SLAM 原始地图不被修改。");hint->setWordWrap(true);layout->addWidget(hint);
 auto form=new QFormLayout;mode_=new QComboBox;mode_->setObjectName("zone_mode");mode_->addItems({"虚拟墙（两点）","矩形禁区（对角两点）","多边形禁区（逐点，完成绘制）"});form->addRow("形状",mode_);
 name_=new QLineEdit("禁行约束");name_->setMaxLength(128);form->addRow("名称",name_);
 width_=new QDoubleSpinBox;width_->setRange(.01,2);width_->setValue(.1);width_->setSuffix(" m");form->addRow("墙宽",width_);
 margin_=new QDoubleSpinBox;margin_->setRange(0,2);margin_->setValue(.05);margin_->setSuffix(" m");form->addRow("额外边距",margin_);layout->addLayout(form);
 auto row=new QHBoxLayout;draw_=new QPushButton("开始绘制");draw_->setObjectName("zone_draw");finish_=new QPushButton("完成绘制");finish_->setObjectName("zone_finish");undo_=new QPushButton("撤回一点");row->addWidget(draw_);row->addWidget(finish_);row->addWidget(undo_);layout->addLayout(row);
 list_=new QListWidget;list_->setObjectName("zone_list");layout->addWidget(list_);
 row=new QHBoxLayout;erase_=new QPushButton("删除选中");toggle_=new QPushButton("启用 / 禁用");row->addWidget(erase_);row->addWidget(toggle_);layout->addLayout(row);
 reviewed_=new QCheckBox("已核对地图、定位及约束位置（载图继承后需重新核对）");reviewed_->setObjectName("zone_reviewed");layout->addWidget(reviewed_);
 row=new QHBoxLayout;save_=new QPushButton("保存并应用");save_->setObjectName("zones_replace");reload_=new QPushButton("重新加载 / 放弃草稿");reload_->setObjectName("zone_reload");row->addWidget(save_);row->addWidget(reload_);layout->addLayout(row);
 connect(draw_,&QPushButton::clicked,this,[this]{picking_=true;vertices_=Json::array();refresh();emit pickRequested();});
 connect(finish_,&QPushButton::clicked,this,[this]{finish();});
 connect(undo_,&QPushButton::clicked,this,[this]{if(!vertices_.empty())vertices_.erase(vertices_.size()-1);refresh();});
 connect(mode_,qOverload<int>(&QComboBox::currentIndexChanged),this,[this]{cancelPick();});
 connect(erase_,&QPushButton::clicked,this,[this]{int i=list_->currentRow();if(i>=0){draft_.erase(i);dirty_=true;rebuild();refresh();}});
 connect(toggle_,&QPushButton::clicked,this,[this]{int i=list_->currentRow();if(i>=0){draft_[i]["enabled"]=!draft_[i].value("enabled",true);dirty_=true;rebuild();refresh();}});
 connect(reload_,&QPushButton::clicked,this,[this]{load();});
 connect(reviewed_,&QCheckBox::toggled,this,[this]{refresh();});
 connect(save_,&QPushButton::clicked,this,[this]{if(!save_->isEnabled())return;emit commandRequested("zones_replace",QString::fromStdString(Json({{"expected_zones_boot",boot_},{"context_id",context_},{"expected_revision",revision_},{"regions",draft_},{"reviewed",true}}).dump()));});
 refresh();
}
void ZonePage::load(){
 if(!valid_)return;context_=state_.at("context_id");boot_=state_.at("boot_id");revision_=state_.at("revision");draft_=state_.at("regions");dirty_=picking_=conflict_=false;vertices_=Json::array();reviewed_->setChecked(false);rebuild();refresh();
}
void ZonePage::updateStatus(const Json &observation,bool authority){
 authority_=authority;valid_=observation.value("quality","")=="VALID"&&observation.contains("value")&&observation.at("value").is_object();
 if(valid_){state_=observation.at("value");const bool changed=context_!=state_.value("context_id","")||boot_!=state_.value("boot_id","");
   if(changed){load();}
   else if(state_.at("revision")!=revision_){
     if(!dirty_||state_.at("regions")==draft_)load();else conflict_=true;
   }
 }
 if(!valid_||!authority_){picking_=false;vertices_=Json::array();}refresh();
}
bool ZonePage::picked(const Json &p){
 if(!picking_||!valid_||!authority_||p.value("frame","")!="map")return false;
 try{double x=p.at("x"),y=p.at("y");if(!std::isfinite(x)||!std::isfinite(y)||std::abs(x)>10000||std::abs(y)>10000||vertices_.size()>=32)return false;vertices_.push_back({x,y});
 if(mode_->currentIndex()<2&&vertices_.size()==2)finish();else refresh();return true;}catch(...){return false;}
}
void ZonePage::finish(){
 try{
 Json points=vertices_;const int mode=mode_->currentIndex();if(mode==1&&points.size()==2){double x=points[0][0],y=points[0][1],xx=points[1][0],yy=points[1][1];points={{x,y},{xx,y},{xx,yy},{x,yy}};}
 Json region={{"id",QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString()},{"name",name_->text().toStdString()},{"type",mode==0?"wall":"polygon"},{"enabled",true},{"points",points},{"width_m",mode==0?width_->value():0.},{"margin_m",margin_->value()}};
 auto next=draft_;next.push_back(region);draft_=astribot_navigation_zones::encodeRegions(astribot_navigation_zones::parseRegions(next));dirty_=true;picking_=false;vertices_=Json::array();reviewed_->setChecked(false);rebuild();refresh();
 }catch(const std::exception &e){status_->setText(QString("形状无效，请撤点修正：%1").arg(e.what()));}
}
void ZonePage::rebuild(){const int row=list_->currentRow();list_->clear();for(const auto &r:draft_)list_->addItem(QString("%1  %2  (%3 个点)").arg(r.value("enabled",true)?"启用":"禁用").arg(QString::fromStdString(r.value("name",std::string()))).arg(r.at("points").size()));list_->setCurrentRow(std::min(row,list_->count()-1));}
void ZonePage::refresh(){
 const bool editable=valid_&&authority_&&state_.value("can_edit",false)&&!conflict_&&!context_.empty();
 draw_->setEnabled(editable&&!picking_&&draft_.size()<64);finish_->setEnabled(editable&&picking_&&vertices_.size()>=(mode_->currentIndex()==2?3u:2u));undo_->setEnabled(picking_&&!vertices_.empty());erase_->setEnabled(editable&&!picking_);toggle_->setEnabled(editable&&!picking_);
 save_->setEnabled(editable&&!picking_&&reviewed_->isChecked()&&(dirty_||state_.value("review_required",false)));reload_->setEnabled(valid_);mode_->setEnabled(!picking_);width_->setEnabled(!picking_);margin_->setEnabled(!picking_);
 status_->setText(!valid_?"禁区状态未连接 / 过期：禁止提交":QString("地图会话：%1\n版本 %2 · %3 · %4\n%5").arg(QString::fromStdString(context_)).arg(revision_).arg(QString::fromStdString(state_.value("reason_code",std::string()))).arg(dirty_?"有未保存草稿":"服务端已保存").arg(conflict_?"版本已变化，请重新加载后编辑":picking_?QString("在地图上点击，已选 %1 点").arg(vertices_.size()):!editable?"请取得控制权，暂停任务并等待停稳":"可以编辑；保存后仍需等待所有消费者生效"));
}
visualization_msgs::msg::MarkerArray ZonePage::markers(const std::string &ns)const{
 visualization_msgs::msg::MarkerArray out;if(!valid_)return out;int id=0;
 auto line=[&](const Json&points,bool polygon,bool draft,bool enabled,double width){if(points.empty())return;visualization_msgs::msg::Marker m;m.header.frame_id="map";m.ns=ns;m.id=id++;m.type=m.LINE_STRIP;m.action=m.ADD;m.pose.orientation.w=1;m.scale.x=std::max(.025,width);m.color.r=1;m.color.g=draft? .8:0;m.color.a=enabled?.85:.25;
 for(const auto&p:points){geometry_msgs::msg::Point q;q.x=p.at(0);q.y=p.at(1);q.z=.08;m.points.push_back(q);}if(polygon)m.points.push_back(m.points.front());out.markers.push_back(m);};
 for(const auto &r:state_.value("regions",Json::array()))line(r.at("points"),r.at("type")=="polygon",false,r.value("enabled",true),r.value("width_m",.03));
 if(dirty_)for(const auto&r:draft_)line(r.at("points"),r.at("type")=="polygon",true,r.value("enabled",true),r.value("width_m",.03));
 line(vertices_,false,true,true,.04);return out;
}
}
