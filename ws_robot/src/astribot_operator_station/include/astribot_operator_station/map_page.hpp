#pragma once
#include <QWidget>
#include <QComboBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
#include <nlohmann/json.hpp>
namespace astribot_operator_station {
class MapPage:public QWidget {
 Q_OBJECT
 nlohmann::json catalog_;bool authority_{false};std::string rendered_;
 QComboBox * maps_,*stations_,*kind_;QLineEdit * directory_,*id_,*station_id_;QSpinBox * floor_;
 QDoubleSpinBox * x_,*y_,*yaw_,*work_x_,*work_y_,*work_yaw_;QCheckBox * transfer_,*work_;QLabel * detail_;
 std::map<std::string,QPushButton *> buttons_;
 void request(const std::string & op);void refresh();
public:
 explicit MapPage(QWidget * parent=nullptr);
 void updateCatalog(const nlohmann::json & observation,bool authority);
 void picked(const nlohmann::json & pose);
signals:
 void commandRequested(QString operation,QString payload);
 void dockSelected(QString pose);
};
}
