#include "astribot_operator_station/map_page.hpp"
#include <gtest/gtest.h>
#include <QApplication>
using namespace astribot_operator_station;
TEST(MapPage, ObserverGateVersionBindingAndTransferStage){
 int argc=1;char name[]="map_page_test";char * argv[]={name,nullptr};QApplication app(argc,argv);MapPage page;
 using Json=nlohmann::json;Json catalog={{"boot_id","mapboot"},{"revision",7},{"state","READY"},{"motion_blocked",false},{"switch_available",true},{"idle_evidence",true},{"stop_evidence",true},
 {"active_map",nullptr},{"transaction",nullptr},{"stations",Json::object()},{"maps",{{"floor1",{{"map_id","floor1"},{"floor",1},{"version","abc123"}}}}}};
 auto button=[&](QString prefix){for(auto b:page.findChildren<QPushButton *>())if(b->text().startsWith(prefix))return b;throw std::runtime_error("missing button");};
 page.updateCatalog({{"quality","VALID"},{"value",catalog}},false);EXPECT_FALSE(button("准备切换")->isEnabled());
 std::string op;Json request;QObject::connect(&page,&MapPage::commandRequested,[&](QString operation,QString payload){op=operation.toStdString();request=Json::parse(payload.toStdString());});
 page.updateCatalog({{"quality","VALID"},{"value",catalog}},true);ASSERT_TRUE(button("准备切换")->isEnabled());button("准备切换")->click();
 EXPECT_EQ(op,"map_switch_begin");EXPECT_EQ(request.at("expected_catalog_boot"),"mapboot");EXPECT_EQ(request.at("expected_revision"),7);EXPECT_EQ(request.at("map_version"),"abc123");
 catalog["revision"]=8;catalog["state"]="WAIT_TRANSFER";catalog["motion_blocked"]=true;catalog["transaction"]={{"transaction_id","tx1"},{"reason_code","MAP.WAIT_TRANSFER"}};
 page.updateCatalog({{"quality","VALID"},{"value",catalog}},true);EXPECT_FALSE(button("准备切换")->isEnabled());ASSERT_TRUE(button("确认已人工")->isEnabled());button("确认已人工")->click();EXPECT_EQ(request.at("transaction_id"),"tx1");
 page.resize(700,900);page.show();app.processEvents();EXPECT_TRUE(page.grab().save("/tmp/astribot_map_page.png"));
 page.updateCatalog({{"quality","STALE"}},true);EXPECT_FALSE(button("确认已人工")->isEnabled());EXPECT_FALSE(button("提交工位")->isEnabled());
}
