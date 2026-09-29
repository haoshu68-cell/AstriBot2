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
namespace astribot_operator_station {
class ZonePage : public QWidget {
 Q_OBJECT
 using Json=nlohmann::json;
 Json state_=Json::object(),draft_=Json::array(),vertices_=Json::array();
 std::string context_,boot_;uint64_t revision_{0};
 bool dirty_{false},picking_{false},valid_{false},authority_{false},conflict_{false};
 QComboBox *mode_;QLineEdit *name_;QDoubleSpinBox *width_,*margin_;
 QListWidget *list_;QLabel *status_;QCheckBox *reviewed_;
 QPushButton *draw_,*finish_,*undo_,*erase_,*toggle_,*save_,*reload_;
 void refresh();void rebuild();void load();void finish();
public:
 explicit ZonePage(QWidget *parent=nullptr);
 void updateStatus(const Json &,bool authority);
 bool picked(const Json &);
 void cancelPick(){picking_=false;vertices_=Json::array();refresh();}
 visualization_msgs::msg::MarkerArray markers(const std::string &ns)const;
signals:
 void commandRequested(QString operation,QString payload);
 void pickRequested();
};
}
