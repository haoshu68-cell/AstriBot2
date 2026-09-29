#pragma once
#include <QWidget>
#include <QComboBox>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QListWidget>
#include <QLabel>
#include <QCheckBox>
#include <QPushButton>
#include <nlohmann/json.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <array>
#include <map>
namespace astribot_operator_station {
class ScenePage:public QWidget {
 Q_OBJECT
 using Json=nlohmann::json;
 Json catalog_=Json::object(),position_,dock_,wait_;
 bool authority_{false},valid_{false};std::string rendered_,pick_mode_;
 int device_version_{0},scene_version_{0};
 QComboBox * type_,*scenes_,*maps_,*device_type_;
 QLineEdit * scene_id_,*scene_name_,*device_id_,*device_name_;
 QListWidget * devices_;QLabel * status_,*coordinates_;
 QDoubleSpinBox * width_,*depth_;QCheckBox * reviewed_,*enabled_,*visible_,*transfer_;
 std::map<std::string,QPushButton *> buttons_;
 void rebuildDevices();void chooseScene();void chooseDevice();void request(const std::string & op);
 void refresh();void showCoordinates();
 Json map()const;Json refs()const;
public:
 explicit ScenePage(QWidget * parent=nullptr);
 void updateCatalog(const Json &,bool authority);
 void cancelPick(){pick_mode_.clear();}
 bool picked(const Json & pose);
 visualization_msgs::msg::MarkerArray markers(const std::string & ns)const;
signals:
 void commandRequested(QString operation,QString payload);
 void pickRequested();void dockSelected(QString pose);void focusRequested(double x,double y);
};
}
