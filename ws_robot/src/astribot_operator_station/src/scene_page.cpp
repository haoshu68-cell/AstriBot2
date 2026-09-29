#include "astribot_operator_station/scene_page.hpp"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QSignalBlocker>
#include <cmath>
namespace astribot_operator_station {
namespace {
void types(QComboBox * box){for(auto item:{std::make_pair("echo","echo"),{"冻干","freeze_dryer"},{"酶标仪","plate_reader"},{"封膜机","sealer"},{"恒温箱开门","incubator_door"}})box->addItem(item.first,item.second);}
QString str(const nlohmann::json & p,const char * k){return QString::fromStdString(p.at(k).get<std::string>());}
}
ScenePage::ScenePage(QWidget * parent):QWidget(parent){
 auto layout=new QVBoxLayout(this);setMinimumSize(0,0);
 status_=new QLabel("场景管理未连接");status_->setWordWrap(true);status_->setTextFormat(Qt::PlainText);layout->addWidget(status_);
 auto form=new QFormLayout;layout->addLayout(form);
 type_=new QComboBox;type_->setObjectName("scene_type");types(type_);form->addRow("业务类型",type_);
 scenes_=new QComboBox;scenes_->setObjectName("scene_instances");form->addRow("已保存场景",scenes_);
 scene_id_=new QLineEdit;scene_id_->setObjectName("scene_id");scene_id_->setPlaceholderText("唯一 ID：字母/数字/_/-");form->addRow("场景 ID",scene_id_);
 scene_name_=new QLineEdit;scene_name_->setObjectName("scene_name");form->addRow("场景名称",scene_name_);
 maps_=new QComboBox;maps_->setObjectName("scene_map");form->addRow("绑定地图及版本",maps_);
 auto button=[&](const char * text,const std::string & op,auto callback){auto b=new QPushButton(text);b->setObjectName(QString::fromStdString(op));layout->addWidget(b);buttons_[op]=b;connect(b,&QPushButton::clicked,this,callback);};
 button("新建场景草稿","new_scene",[this]{scene_version_=0;scene_id_->clear();scene_name_->clear();for(int i=0;i<devices_->count();++i)devices_->item(i)->setCheckState(Qt::Unchecked);});
 devices_=new QListWidget;devices_->setObjectName("scene_devices");devices_->setMaximumHeight(180);layout->addWidget(new QLabel("设备：勾选以绑定场景，选中以编辑；待复核标记不能直接用于作业"));layout->addWidget(devices_);
 auto device_form=new QFormLayout;layout->addLayout(device_form);
 device_id_=new QLineEdit;device_id_->setObjectName("device_id");device_form->addRow("设备 ID",device_id_);
 device_name_=new QLineEdit;device_name_->setObjectName("device_name");device_form->addRow("设备名称",device_name_);
 device_type_=new QComboBox;types(device_type_);device_form->addRow("设备类型",device_type_);
 width_=new QDoubleSpinBox;depth_=new QDoubleSpinBox;for(auto b:{width_,depth_}){b->setRange(.01,100);b->setDecimals(3);b->setValue(.6);b->setSuffix(" m");}device_form->addRow("轮廓长（设备 X）",width_);device_form->addRow("轮廓宽（设备 Y）",depth_);
 coordinates_=new QLabel("尚未标记设备位置");coordinates_->setObjectName("device_coordinates");coordinates_->setWordWrap(true);coordinates_->setTextFormat(Qt::PlainText);layout->addWidget(coordinates_);
 reviewed_=new QCheckBox("已人工复核地图标记及停靠位（不代表机械臂标定）");enabled_=new QCheckBox("设备启用");enabled_->setChecked(true);visible_=new QCheckBox("显示设备标记");visible_->setChecked(true);layout->addWidget(reviewed_);layout->addWidget(enabled_);layout->addWidget(visible_);
 button("新增设备 / 放弃当前编辑","new_device",[this]{device_version_=0;device_id_->clear();device_name_->clear();position_=nullptr;dock_=nullptr;wait_=nullptr;reviewed_->setChecked(false);enabled_->setChecked(true);showCoordinates();});
 for(auto item:{std::make_pair("地图标记设备位置和朝向","pose"),{"地图标记停靠位","dock"},{"地图标记等待位","wait"},{"地图点击选择已有设备","select"}}){std::string mode=item.second;button(item.first,"pick_"+mode,[this,mode]{pick_mode_=mode;status_->setText(mode=="select"?"在设备标记附近点击，选择设备":"在地图上点击并拖动设置朝向；仅编辑草稿");emit pickRequested();});}
 button("取消地图标记","cancel_pick",[this]{pick_mode_.clear();refresh();});
 button("保存设备新版本","device_put",[this]{request("device_put");});
 button("重新读取选中设备 / 放弃编辑","reload_device",[this]{chooseDevice();});
 button("定位显示选中设备","focus_device",[this]{if(!position_.is_null())emit focusRequested(position_.at("x"),position_.at("y"));});
 button("将已复核停靠位加入导航草稿","use_device",[this]{
 if(!valid_||!catalog_.value("scene_ready",false)||catalog_.value("motion_blocked",true)||catalog_.value("active_scene",Json()).is_null())return;
 auto key=device_id_->text().toStdString();if(!catalog_.at("devices").contains(key))return;const auto & d=catalog_.at("devices").at(key);
 bool bound=false;for(const auto & ref:catalog_.at("active_scene").at("devices"))if(ref.at("device_id")==key&&ref.at("version")==d.at("version"))bound=true;
 if(bound&&d.value("enabled",false)&&d.value("reviewed",false)&&!d.at("dock_pose").is_null())emit dockSelected(QString::fromStdString(d.at("dock_pose").dump()));
 else status_->setText("设备需绑定到活动场景、启用并完成复核，且具有停靠位");});
 button("保存场景与设备绑定","scene_put",[this]{request("scene_put");});
 transfer_=new QCheckBox("跨楼层/人工转运：使用已有地图事务确认流程");layout->addWidget(transfer_);
 button("加载选中场景与地图","scene_load",[this]{request("scene_load");});
 layout->addWidget(new QLabel("加载不执行业务动作；设备开门、上下料等能力尚未在此页接入。"));
 connect(width_,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this]{reviewed_->setChecked(false);});
 connect(depth_,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this]{reviewed_->setChecked(false);});
 connect(type_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this]{rendered_.clear();if(valid_)updateCatalog({{"quality","VALID"},{"value",catalog_}},authority_);});
 connect(scenes_,QOverload<int>::of(&QComboBox::activated),this,[this]{chooseScene();});
 connect(maps_,QOverload<int>::of(&QComboBox::activated),this,[this]{pick_mode_.clear();position_=nullptr;dock_=nullptr;wait_=nullptr;device_version_=0;device_id_->clear();device_name_->clear();reviewed_->setChecked(false);rebuildDevices();showCoordinates();});
 connect(devices_,&QListWidget::currentRowChanged,this,[this]{chooseDevice();});refresh();
}
ScenePage::Json ScenePage::map()const{auto id=maps_->currentData().toString().toStdString();return catalog_.value("maps",Json::object()).value(id,Json::object());}
ScenePage::Json ScenePage::refs()const{Json out=Json::array();for(int i=0;i<devices_->count();++i){auto item=devices_->item(i);if(item->checkState()==Qt::Checked){auto d=Json::parse(item->data(Qt::UserRole).toString().toStdString());out.push_back({{"device_id",d.at("device_id")},{"version",d.at("version")}});}}return out;}
void ScenePage::rebuildDevices(){
 QSignalBlocker block(devices_);auto selected=device_id_->text();devices_->clear();auto m=map();if(m.empty())return;
 for(const auto & d:catalog_.value("devices",Json::object()))if(d.at("map_id")==m.at("map_id")&&d.at("map_version")==m.at("version")){
 auto item=new QListWidgetItem(str(d,"name")+QString(" · v%1 · ").arg(d.at("version").get<int>())+(d.value("enabled",true)?d.value("reviewed",false)?"已复核":"待复核":"已停用"),devices_);item->setData(Qt::UserRole,QString::fromStdString(d.dump()));item->setFlags(item->flags()|Qt::ItemIsUserCheckable);item->setCheckState(Qt::Unchecked);if(str(d,"device_id")==selected)devices_->setCurrentItem(item);
 }
}
void ScenePage::chooseScene(){
 auto id=scenes_->currentData().toString().toStdString();if(!catalog_.value("scenes",Json::object()).contains(id))return;const auto & s=catalog_.at("scenes").at(id);
 pick_mode_.clear();position_=nullptr;dock_=nullptr;wait_=nullptr;device_version_=0;device_id_->clear();device_name_->clear();reviewed_->setChecked(false);showCoordinates();
 scene_id_->setText(str(s,"scene_id"));scene_name_->setText(str(s,"name"));scene_version_=s.at("version");maps_->setCurrentIndex(maps_->findData(str(s,"map_id")));rebuildDevices();
 for(int i=0;i<devices_->count();++i){auto d=Json::parse(devices_->item(i)->data(Qt::UserRole).toString().toStdString());for(const auto & ref:s.at("devices"))if(ref.at("device_id")==d.at("device_id")&&ref.at("version")==d.at("version"))devices_->item(i)->setCheckState(Qt::Checked);}
 refresh();
}
void ScenePage::chooseDevice(){auto item=devices_->currentItem();if(!item)return;auto d=Json::parse(item->data(Qt::UserRole).toString().toStdString());device_id_->setText(str(d,"device_id"));device_name_->setText(str(d,"name"));device_type_->setCurrentIndex(device_type_->findData(str(d,"type")));device_version_=d.at("version");position_=d.at("pose");dock_=d.at("dock_pose");wait_=d.at("wait_pose");width_->setValue(d.at("width"));depth_->setValue(d.at("depth"));reviewed_->setChecked(d.at("reviewed"));enabled_->setChecked(d.at("enabled"));showCoordinates();}
void ScenePage::showCoordinates(){
 auto describe=[](const Json & p){return p.is_null()?QString("未设置"):QString("X %1 m，Y %2 m，朝向 %3°").arg(p.at("x").get<double>(),0,'f',3).arg(p.at("y").get<double>(),0,'f',3).arg(p.at("yaw").get<double>()*180.0/3.141592653589793,0,'f',1);};
 coordinates_->setText("设备："+describe(position_)+"\n停靠："+describe(dock_)+"\n等待："+describe(wait_));
}
bool ScenePage::picked(const Json & p){
 if(pick_mode_.empty())return false;
 auto mode=pick_mode_;pick_mode_.clear();if(!authority_||p.value("frame","")!="map")return true;
 auto selected_map=map();auto active=catalog_.value("active_map",Json());if(!catalog_.value("active_map_ready",!catalog_.value("motion_blocked",true))||active.is_null()||selected_map.empty()||selected_map.at("map_id")!=active.at("map_id")||selected_map.at("version")!=active.at("version")){status_->setText("请先加载所选地图并完成定位，再标记设备；禁止将当前地图坐标保存到另一地图");return true;}
 try{for(auto k:{"x","y","yaw"})if(!std::isfinite(p.at(k).get<double>()))return true;
 if(mode=="select"){double nearest=1.;int row=-1;for(int i=0;i<devices_->count();++i){auto d=Json::parse(devices_->item(i)->data(Qt::UserRole).toString().toStdString());double distance=std::hypot(d.at("pose").at("x").get<double>()-p.at("x").get<double>(),d.at("pose").at("y").get<double>()-p.at("y").get<double>());if(distance<nearest){nearest=distance;row=i;}}if(row>=0){devices_->setCurrentRow(row);chooseDevice();}return true;}
 if(mode=="pose")position_=p;else if(mode=="dock")dock_=p;else wait_=p;reviewed_->setChecked(false);showCoordinates();
 }catch(...){status_->setText("标记坐标非法");}return true;
}
void ScenePage::updateCatalog(const Json & observation,bool authority){
 valid_=observation.value("quality","")=="VALID";authority_=valid_&&authority;if(!valid_){pick_mode_.clear();refresh();return;}
 catalog_=observation.at("value");auto key=catalog_.at("boot_id").get<std::string>()+":"+std::to_string(catalog_.at("revision").get<uint64_t>());
 if(key!=rendered_){rendered_=key;auto selected=scenes_->currentData();auto selected_map=maps_->currentData();QSignalBlocker a(scenes_),b(maps_);scenes_->clear();maps_->clear();
 for(const auto & s:catalog_.value("scenes",Json::object()))if(str(s,"type")==type_->currentData())scenes_->addItem(str(s,"name")+QString(" v%1").arg(s.at("version").get<int>()),str(s,"scene_id"));
 for(const auto & m:catalog_.at("maps"))maps_->addItem(str(m,"map_id")+" · "+str(m,"version").left(8),str(m,"map_id"));
 auto i=scenes_->findData(selected);if(i>=0)scenes_->setCurrentIndex(i);i=maps_->findData(selected_map);if(i>=0)maps_->setCurrentIndex(i);
 auto checked=refs();rebuildDevices();for(int row=0;row<devices_->count();++row){auto d=Json::parse(devices_->item(row)->data(Qt::UserRole).toString().toStdString());for(const auto & ref:checked)if(ref.at("device_id")==d.at("device_id")&&ref.at("version")==d.at("version"))devices_->item(row)->setCheckState(Qt::Checked);}
 }
 refresh();
}
void ScenePage::refresh(){
 bool writable=authority_&&catalog_.value("idle_evidence",false)&&catalog_.value("state",std::string())=="READY";for(auto & entry:buttons_)entry.second->setEnabled(writable);
 buttons_.at("focus_device")->setEnabled(valid_&&!position_.is_null());buttons_.at("reload_device")->setEnabled(valid_);buttons_.at("cancel_pick")->setEnabled(true);
 buttons_.at("scene_load")->setEnabled(writable&&scenes_->currentIndex()>=0&&(!catalog_.value("motion_blocked",true)||catalog_.value("switch_available",false)));
 if(!writable)pick_mode_.clear();
 if(!valid_){status_->setText("场景管理未连接或状态过期；设备标记停止显示，编辑与加载禁用");return;}
 const auto active=catalog_.value("active_scene",Json());status_->setText("活动场景："+(active.is_null()?QString("未加载"):str(active,"name"))+"\n地图事务："+QString::fromStdString(catalog_.value("state","UNKNOWN"))+"\n"+(catalog_.value("scene_ready",false)?"场景上下文有效；业务执行能力独立检查":"场景上下文待验证/已变化")+"\n存储："+QString::fromStdString(catalog_.value("storage_root",std::string("未知")))+"\n保存后重新选择条目可读取最新版本；设备修改后需重新保存并加载场景。过期的设备绑定不会自动勾选，需人工重新勾选确认。");
 if(!pick_mode_.empty())status_->setText(status_->text()+"\n正在地图标记："+QString::fromStdString(pick_mode_)+"，拖动设置朝向；可取消标记。");
}
void ScenePage::request(const std::string & op){
 if(!authority_)return;
 try{Json p={{"expected_catalog_boot",catalog_.at("boot_id")},{"expected_revision",catalog_.at("revision")}};auto m=map();
 if(op=="scene_load"){auto key=scenes_->currentData().toString().toStdString();const auto & scene=catalog_.at("scenes").at(key);p["scene_id"]=key;p["scene_version"]=scene.at("version");p["manual_transfer"]=transfer_->isChecked();}
 else {if(m.empty())throw std::runtime_error("请先导入并选择地图");p["map_id"]=m.at("map_id");p["map_version"]=m.at("version");
 if(op=="device_put"){if(position_.is_null())throw std::runtime_error("请先在地图标记设备");p.update({{"device_id",device_id_->text().toStdString()},{"name",device_name_->text().toStdString()},{"type",device_type_->currentData().toString().toStdString()},{"expected_device_version",device_version_},{"pose",position_},{"dock_pose",dock_},{"wait_pose",wait_},{"width",width_->value()},{"depth",depth_->value()},{"reviewed",reviewed_->isChecked()},{"enabled",enabled_->isChecked()}});}
 else p.update({{"scene_id",scene_id_->text().toStdString()},{"name",scene_name_->text().toStdString()},{"type",type_->currentData().toString().toStdString()},{"expected_scene_version",scene_version_},{"devices",refs()}});}
 emit commandRequested(QString::fromStdString(op),QString::fromStdString(p.dump()));
 }catch(const std::exception & e){status_->setText(e.what());}
}
visualization_msgs::msg::MarkerArray ScenePage::markers(const std::string & ns)const{
 using M=visualization_msgs::msg::Marker;visualization_msgs::msg::MarkerArray out;
 if(!valid_||catalog_.value("motion_blocked",true)||!visible_->isChecked()||catalog_.value("active_map",Json()).is_null())return out;
 const auto & active=catalog_.at("active_map");int index=0;
 for(const auto & d:catalog_.value("devices",Json::object()))if(d.at("map_id")==active.at("map_id")&&d.at("map_version")==active.at("version")){
 M m;m.header.frame_id="map";m.ns=ns;m.id=index++;m.action=M::ADD;m.type=M::CUBE;m.lifetime.sec=1;const auto & p=d.at("pose");double yaw=p.at("yaw");m.pose.position.x=p.at("x");m.pose.position.y=p.at("y");m.pose.position.z=.04;m.pose.orientation.z=std::sin(yaw/2);m.pose.orientation.w=std::cos(yaw/2);m.scale.x=d.at("width");m.scale.y=d.at("depth");m.scale.z=.08;m.color.a=.6;m.color.r=d.value("reviewed",false)?0.1:1.;m.color.g=d.value("enabled",true)?.7:.1;m.color.b=str(d,"device_id")==device_id_->text()?1.:.2;out.markers.push_back(m);
 m.id=index++;m.type=M::TEXT_VIEW_FACING;m.pose.position.z=.6;m.scale.z=.2;m.color.a=1;m.text=d.at("name").get<std::string>()+" v"+std::to_string(d.at("version").get<int>());out.markers.push_back(m);
 for(auto k:{"dock_pose","wait_pose"})if(!d.at(k).is_null()){const auto & q=d.at(k);m.id=index++;m.type=M::ARROW;m.pose.position.x=q.at("x");m.pose.position.y=q.at("y");m.pose.position.z=.1;double angle=q.at("yaw");m.pose.orientation.z=std::sin(angle/2);m.pose.orientation.w=std::cos(angle/2);m.scale.x=.4;m.scale.y=.06;m.scale.z=.06;out.markers.push_back(m);}
 }
 auto selected=map();
 if(!position_.is_null()&&!selected.empty()&&selected.at("map_id")==active.at("map_id")&&selected.at("version")==active.at("version")){
 M draft;draft.header.frame_id="map";draft.ns=ns;draft.id=index++;draft.type=M::CUBE;draft.lifetime.sec=1;
 draft.pose.position.x=position_.at("x");draft.pose.position.y=position_.at("y");draft.pose.position.z=.12;
 const double angle=position_.at("yaw");draft.pose.orientation.z=std::sin(angle/2);draft.pose.orientation.w=std::cos(angle/2);
 draft.scale.x=width_->value();draft.scale.y=depth_->value();draft.scale.z=.04;draft.color.r=0.;draft.color.g=1.;draft.color.b=1.;draft.color.a=.35;out.markers.push_back(draft);
 draft.id=index++;draft.type=M::TEXT_VIEW_FACING;draft.pose.position.z=.9;draft.scale.z=.17;draft.color.a=1.;draft.text="草稿 / "+device_name_->text().toStdString();out.markers.push_back(draft);
 }
 return out;
}
}
