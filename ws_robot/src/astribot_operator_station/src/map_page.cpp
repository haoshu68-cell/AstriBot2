#include "astribot_operator_station/map_page.hpp"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QFileDialog>
namespace astribot_operator_station {
using Json=nlohmann::json;
MapPage::MapPage(QWidget * parent):QWidget(parent){
 auto outer=new QVBoxLayout(this);auto scroll=new QScrollArea;scroll->setWidgetResizable(true);outer->addWidget(scroll);auto content=new QWidget;scroll->setWidget(content);auto layout=new QVBoxLayout(content);auto form=new QFormLayout;layout->addLayout(form);
 detail_=new QLabel("地图管理未连接");detail_->setWordWrap(true);detail_->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Maximum);detail_->setTextFormat(Qt::PlainText);layout->addWidget(detail_);
 maps_=new QComboBox;maps_->setObjectName("catalog_maps");form->addRow("地图版本",maps_);
 stations_=new QComboBox;form->addRow("工位版本",stations_);
 directory_=new QLineEdit;directory_->setPlaceholderText("机器人上已保存的 SLAM 会话目录");
 auto directory_row=new QHBoxLayout;directory_row->addWidget(directory_);
 auto browse=new QPushButton("选择目录");browse->setObjectName("browse_map_directory");directory_row->addWidget(browse);
 form->addRow("导入目录（机器人）",directory_row);
 connect(browse,&QPushButton::clicked,this,[this]{
   const auto selected=QFileDialog::getExistingDirectory(this,"选择已保存的 SLAM 会话目录",directory_->text());
   if(!selected.isEmpty())directory_->setText(selected);
 });
 id_=new QLineEdit;id_->setObjectName("catalog_map_id");form->addRow("新地图 ID",id_);floor_=new QSpinBox;floor_->setRange(-20,200);form->addRow("楼层",floor_);
 auto button=[&](const QString & label,const std::string & op){auto b=new QPushButton(label);b->setEnabled(false);buttons_[op]=b;layout->addWidget(b);connect(b,&QPushButton::clicked,this,[this,op]{request(op);});};
 button("校验并归档已保存地图","map_import");
 station_id_=new QLineEdit;form->addRow("工位 ID",station_id_);kind_=new QComboBox;kind_->addItems({"shelf","workstation","standby","handover"});form->addRow("工位用途",kind_);
 auto coordinates=[&](QString label,QDoubleSpinBox *& x,QDoubleSpinBox *& y,QDoubleSpinBox *& yaw){auto row=new QHBoxLayout;
 for(auto entry:{&x,&y,&yaw}){*entry=new QDoubleSpinBox;(*entry)->setRange(-100000,100000);(*entry)->setDecimals(6);row->addWidget(*entry);}form->addRow(label,row);};
 coordinates("停靠 X / Y / yaw(rad)",x_,y_,yaw_);work_=new QCheckBox("独立作业目标（仅平面位置，不直接执行）");form->addRow(work_);
 coordinates("作业 X / Y / yaw(rad)",work_x_,work_y_,work_yaw_);
 button("提交工位新版本","station_put");button("将当前工位停靠点加入导航草稿","use_station");
 transfer_=new QCheckBox("人工换层（到交接位后操作；确认运输姿态）");layout->addWidget(transfer_);
 button("准备切换所选地图 / 进入人工转运等待","map_switch_begin");button("确认已人工转运，加载并验证目标地图","map_transfer_confirm");button("放弃尚未转运的事务","map_abort");button("人工核对后重新加载并验证（不恢复动作）","map_recover");
 connect(maps_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this]{stations_->clear();if(catalog_.contains("stations"))for(const auto & s:catalog_.at("stations"))if(s.at("map_id")==maps_->currentData().toString().toStdString())stations_->addItem(QString::fromStdString(s.at("station_id").get<std::string>())+" v"+QString::number(s.at("version").get<int>()),QString::fromStdString(s.dump()));refresh();});
 connect(stations_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this]{if(stations_->currentIndex()<0)return;try{auto s=Json::parse(stations_->currentData().toString().toStdString());station_id_->setText(QString::fromStdString(s.at("station_id")));kind_->setCurrentText(QString::fromStdString(s.at("kind")));picked(s.at("dock_pose"));work_->setChecked(!s.at("work_pose").is_null());if(work_->isChecked()){work_x_->setValue(s.at("work_pose").at("x"));work_y_->setValue(s.at("work_pose").at("y"));work_yaw_->setValue(s.at("work_pose").at("yaw"));}}catch(...){}refresh();});
}
void MapPage::picked(const Json & p){try{if(p.at("frame")!="map")return;x_->setValue(p.at("x"));y_->setValue(p.at("y"));yaw_->setValue(p.at("yaw"));}catch(...) {}}
void MapPage::updateCatalog(const Json & observation,bool authority){
 authority_=authority&&observation.value("quality","")=="VALID";
 if(observation.value("quality","")!="VALID"){detail_->setText("地图管理未连接/过期；禁止修改及切换");authority_=false;for(auto & b:buttons_)b.second->setEnabled(false);return;}
 catalog_=observation.at("value");auto key=catalog_.at("boot_id").get<std::string>()+":"+std::to_string(catalog_.at("revision").get<uint64_t>());
 if(key!=rendered_){rendered_=key;auto selected=maps_->currentData().toString();maps_->clear();for(const auto & m:catalog_.at("maps"))maps_->addItem(QString::fromStdString(m.at("map_id"))+" · F"+QString::number(m.at("floor").get<int>())+" · "+QString::fromStdString(m.at("version").get<std::string>().substr(0,8)),QString::fromStdString(m.at("map_id")));auto index=maps_->findData(selected);if(index>=0)maps_->setCurrentIndex(index);}
 QString description="事务："+QString::fromStdString(catalog_.value("state","UNKNOWN"));if(!catalog_.at("transaction").is_null())description+="\n"+QString::fromStdString(catalog_.at("transaction").value("reason_code",""));
 if(!catalog_.value("switch_available",false))description+="\n地图切换适配器未接入；归档和工位编辑可用。";
 description+="\n活动地图："+QString(catalog_.at("active_map").is_null()?"尚未选择":catalog_.value("active_map_ready",false)?"已验证":"未确认，禁止新运动");
 description+="\n流程：选择已存会话目录 → 校验并归档 → 选择版本 → 准备切换 →（跨楼层时）人工确认 → 适配器加载并验证";
 description+="\n停稳证据："+QString(catalog_.value("stop_evidence",false)?"已观测":"未知/未停稳");detail_->setText(description);refresh();
}
void MapPage::refresh(){for(auto & [op,b]:buttons_)b->setEnabled(authority_);if(catalog_.empty())return;
 bool selected=maps_->currentIndex()>=0;bool blocked=catalog_.value("motion_blocked",true);auto state=catalog_.value("state","");
 buttons_.at("map_import")->setEnabled(authority_&&!blocked);
 buttons_.at("station_put")->setEnabled(authority_&&selected);
 buttons_.at("map_switch_begin")->setEnabled(authority_&&selected&&!blocked&&catalog_.value("switch_available",false)&&catalog_.value("idle_evidence",false));
 buttons_.at("map_transfer_confirm")->setEnabled(authority_&&state=="WAIT_TRANSFER"&&catalog_.value("idle_evidence",false));
 buttons_.at("map_abort")->setEnabled(authority_&&state=="WAIT_TRANSFER"&&!catalog_.at("active_map").is_null()&&catalog_.value("active_map_ready",false));
 buttons_.at("map_recover")->setEnabled(authority_&&state=="RECOVERY_REQUIRED"&&catalog_.value("switch_available",false)&&catalog_.value("idle_evidence",false));
 buttons_.at("use_station")->setEnabled(authority_&&!blocked&&stations_->currentIndex()>=0&&!catalog_.at("active_map").is_null()&&catalog_.at("active_map").at("map_id")==maps_->currentData().toString().toStdString());
}
void MapPage::request(const std::string & op){try{
 if(!authority_)return;
 Json p={{"expected_catalog_boot",catalog_.at("boot_id")},{"expected_revision",catalog_.at("revision")}};
 auto map_id=maps_->currentData().toString().toStdString();
 if(op=="map_import"){p["map_id"]=id_->text().toStdString();p["directory"]=directory_->text().toStdString();p["floor"]=floor_->value();}
 else if(op=="station_put"||op=="map_switch_begin"){
 const auto & m=catalog_.at("maps").at(map_id);p["map_id"]=map_id;p["map_version"]=m.at("version");
 if(op=="map_switch_begin")p["manual_transfer"]=transfer_->isChecked();else{
 p["station_id"]=station_id_->text().toStdString();p["kind"]=kind_->currentText().toStdString();p["dock_pose"]={{"frame","map"},{"x",x_->value()},{"y",y_->value()},{"yaw",yaw_->value()}};
 p["work_pose"]=work_->isChecked()?Json{{"frame","map"},{"x",work_x_->value()},{"y",work_y_->value()},{"yaw",work_yaw_->value()}}:Json();auto key=map_id+"/"+station_id_->text().toStdString();p["expected_station_version"]=catalog_.at("stations").contains(key)?catalog_.at("stations").at(key).at("version"):Json(0);}}
 else if(op=="use_station"){auto station=Json::parse(stations_->currentData().toString().toStdString());emit dockSelected(QString::fromStdString(station.at("dock_pose").dump()));return;}
 else p["transaction_id"]=catalog_.at("transaction").at("transaction_id");
 emit commandRequested(QString::fromStdString(op),QString::fromStdString(p.dump()));
 }catch(const std::exception & e){detail_->setText(QString::fromUtf8(e.what()));}}
}
