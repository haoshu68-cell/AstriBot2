#include "astribot_operator_station/zone_page.hpp"
#include <QApplication>
#include <gtest/gtest.h>
using namespace astribot_operator_station;
TEST(ZonePage, DrawingRevisionConflictMapSwitchAndObserver){
 int argc=1;char name[]="zones";char *argv[]={name,nullptr};QApplication app(argc,argv);ZonePage page;using J=nlohmann::json;
 J status={{"boot_id","b"},{"context_id","mapping:s"},{"revision",0},{"regions",J::array()},{"ready",true},{"can_edit",true},{"review_required",false},{"reason_code","ZONES.READY"}};
 auto update=[&](bool auth=true){page.updateStatus({{"quality","VALID"},{"value",status}},auth);};auto button=[&](const char *n){return page.findChild<QPushButton *>(n);};update(false);EXPECT_FALSE(button("zone_draw")->isEnabled());
 update();button("zone_draw")->click();EXPECT_FALSE(page.picked({{"frame","odom"},{"x",0},{"y",0}}));EXPECT_TRUE(page.picked({{"frame","map"},{"x",0},{"y",0}}));EXPECT_TRUE(page.picked({{"frame","map"},{"x",1},{"y",1}}));
 EXPECT_EQ(page.findChild<QListWidget *>("zone_list")->count(),1);EXPECT_FALSE(button("zones_replace")->isEnabled());page.findChild<QCheckBox *>("zone_reviewed")->setChecked(true);EXPECT_TRUE(button("zones_replace")->isEnabled());
 J request;QObject::connect(&page,&ZonePage::commandRequested,[&](QString op,QString p){EXPECT_EQ(op,"zones_replace");request=J::parse(p.toStdString());});button("zones_replace")->click();EXPECT_EQ(request.at("regions").size(),1u);EXPECT_EQ(request.at("context_id"),"mapping:s");
 status["revision"]=1;update();EXPECT_FALSE(button("zones_replace")->isEnabled());button("zone_reload")->click();EXPECT_EQ(page.findChild<QListWidget *>("zone_list")->count(),0);
 page.findChild<QComboBox *>("zone_mode")->setCurrentIndex(1);button("zone_draw")->click();page.picked({{"frame","map"},{"x",0},{"y",0}});page.picked({{"frame","map"},{"x",2},{"y",1}});page.findChild<QCheckBox *>("zone_reviewed")->setChecked(true);button("zones_replace")->click();EXPECT_EQ(request.at("regions").at(0).at("points").size(),4u);
 page.resize(610,700);page.show();app.processEvents();EXPECT_TRUE(page.grab().save("/tmp/astribot_zone_page.png"));
 status["context_id"]="map:other:v1";update();EXPECT_EQ(page.findChild<QListWidget *>("zone_list")->count(),0);page.updateStatus({{"quality","STALE"}},true);EXPECT_FALSE(button("zone_draw")->isEnabled());EXPECT_TRUE(page.markers("test").markers.empty());
}
