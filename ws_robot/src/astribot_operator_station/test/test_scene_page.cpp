#include "astribot_operator_station/scene_page.hpp"
#include <gtest/gtest.h>
#include <QApplication>
using namespace astribot_operator_station;
TEST(ScenePage, PersistentSelectionMarkingAndObserverGate) {
 int argc=1;char name[]="scenes";char *argv[]={name,nullptr};QApplication app(argc,argv);
 using J=nlohmann::json;ScenePage page;
 J pose={{"frame","map"},{"x",1.},{"y",2.},{"yaw",.4}};
 J d={{"device_id","echo01"},{"name","echo 01"},{"type","echo"},{"map_id","lab"},{"map_version","map1"},{"version",1},{"pose",pose},{"dock_pose",pose},{"wait_pose",nullptr},{"width",.6},{"depth",.8},{"reviewed",true},{"enabled",true}};
 J s={{"scene_id","echo_lab"},{"name","echo 实验室"},{"type","echo"},{"map_id","lab"},{"map_version","map1"},{"version",1},{"devices",J::array({{{"device_id","echo01"},{"version",1}}})}};
 J c={{"boot_id","boot"},{"revision",3},{"state","READY"},{"motion_blocked",false},{"idle_evidence",true},{"scene_ready",true},{"active_map",{{"map_id","lab"},{"version","map1"}}},{"active_scene",s},{"maps",{{"lab",{{"map_id","lab"},{"version","map1"}}}}},{"devices",{{"echo01",d}}},{"scenes",{{"echo_lab",s}}}};
 auto update=[&](bool auth){page.updateCatalog({{"quality","VALID"},{"value",c}},auth);};
 auto button=[&](const char *key){return page.findChild<QPushButton *>(key);};
 update(false);ASSERT_EQ(page.findChild<QComboBox *>("scene_type")->count(),5);EXPECT_FALSE(button("device_put")->isEnabled());EXPECT_EQ(page.markers("test").markers.size(),3u);
 update(true);auto scenes=page.findChild<QComboBox *>("scene_instances");QMetaObject::invokeMethod(scenes,"activated",Q_ARG(int,0));
 EXPECT_EQ(page.findChild<QLineEdit *>("scene_id")->text(),"echo_lab");
 page.findChild<QListWidget *>("scene_devices")->setCurrentRow(0);
 J request;QString operation,dock;QObject::connect(&page,&ScenePage::commandRequested,[&](QString op,QString text){operation=op;request=J::parse(text.toStdString());});
 QObject::connect(&page,&ScenePage::dockSelected,[&](QString text){dock=text;});
 button("use_device")->click();EXPECT_EQ(J::parse(dock.toStdString()),pose);
 button("pick_pose")->click();auto changed=pose;changed["x"]=3.;EXPECT_TRUE(page.picked(changed));EXPECT_FALSE(page.picked(changed));
 button("device_put")->click();EXPECT_EQ(operation,"device_put");EXPECT_EQ(request.at("pose").at("x"),3.);EXPECT_EQ(request.at("expected_device_version"),1);EXPECT_FALSE(request.at("reviewed").get<bool>());
 button("scene_load")->click();EXPECT_EQ(request.at("scene_version"),1);EXPECT_EQ(request.at("scene_id"),"echo_lab");
 ScenePage reopened;reopened.updateCatalog({{"quality","VALID"},{"value",c}},true);EXPECT_EQ(reopened.markers("test").markers.size(),3u);
 c["motion_blocked"]=true;c["active_map_ready"]=false;c["switch_available"]=true;update(true);
 EXPECT_TRUE(button("scene_load")->isEnabled());EXPECT_TRUE(page.markers("test").markers.empty());
 button("pick_pose")->click();EXPECT_TRUE(page.picked(changed));
 c["motion_blocked"]=false;c["active_map_ready"]=true;update(true);
 page.resize(620,1500);page.show();app.processEvents();EXPECT_TRUE(page.grab().save("/tmp/astribot_scene_page.png"));
 page.updateCatalog({{"quality","STALE"}},true);EXPECT_TRUE(page.markers("test").markers.empty());EXPECT_FALSE(button("scene_load")->isEnabled());
}
