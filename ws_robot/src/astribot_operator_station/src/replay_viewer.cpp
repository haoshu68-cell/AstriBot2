#include "astribot_operator_station/evidence.hpp"
#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QProcess>
#include <QProcessEnvironment>
#include <QTemporaryDir>
#include <QFile>
#include <QTextStream>
#include <QTimer>
#include <QUuid>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <functional>
using Json=nlohmann::json;
int main(int argc,char ** argv) {
 QApplication app(argc,argv);
 if(argc!=2){std::cerr<<"replay_viewer INCIDENT_DIRECTORY\n";return 2;}
 try {
   auto root=std::filesystem::canonical(argv[1]);auto bag=root/"bag";
   if(!std::filesystem::is_directory(bag))throw std::runtime_error("Missing bag directory");
   auto events=astribot_operator_station::read_events(root/"events.jsonl");
   if(events.size()>100000)throw std::runtime_error("More than 100000 events; export a smaller incident interval");
   std::string map_topic="/map",fixed_frame="map";bool map_found=false;
   {rosbag2_cpp::Reader reader;reader.open(bag.string());bool has_map=false,has_nav_map=false;
     for(const auto & topic:reader.get_all_topics_and_types()){has_map=has_map||(topic.name=="/map"&&topic.type=="nav_msgs/msg/OccupancyGrid");has_nav_map=has_nav_map||(topic.name=="/map_nav"&&topic.type=="nav_msgs/msg/OccupancyGrid");}
     if(!has_map&&has_nav_map)map_topic="/map_nav";
     for(size_t count=0;count<20000&&reader.has_next();++count){auto message=reader.read_next();if(message->topic_name==map_topic){
       rclcpp::SerializedMessage serialized(*message->serialized_data);rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> decoder;nav_msgs::msg::OccupancyGrid map;decoder.deserialize_message(&serialized,&map);
       if(!map.header.frame_id.empty())fixed_frame=map.header.frame_id;
       map_found=true;break;
     }}
   }
   QWidget window;window.setWindowTitle("OFFLINE · 只读 3D 与关键参数回放");window.resize(950,750);
   auto layout=new QVBoxLayout(&window);auto label=new QLabel("显示白名单：TF/地图/路径/里程计。历史控制命令不发布；跳转会重建 RViz 和历史缓存。");label->setWordWrap(true);layout->addWidget(label);
   auto frame=new QLineEdit(QString::fromStdString(fixed_frame));layout->addWidget(new QLabel(map_found?"地图参考系（修改后跳转重建）":"未找到已录地图；请核对参考系，缺失数据不会补造"));layout->addWidget(frame);
   auto row=new QHBoxLayout;layout->addLayout(row);auto pause=new QPushButton("暂停");auto play=new QPushButton("播放");auto seek=new QPushButton("跳转并重建");
   auto seconds=new QDoubleSpinBox;seconds->setRange(0,1e7);seconds->setSuffix(" s");auto rate=new QDoubleSpinBox;rate->setRange(.1,4);rate->setValue(1);rate->setSingleStep(.1);rate->setSuffix(" x");
   for(auto b:{pause,play,seek})row->addWidget(b);
   row->addWidget(seconds);row->addWidget(rate);
   auto list=new QListWidget;layout->addWidget(list);auto details=new QPlainTextEdit;details->setReadOnly(true);details->setMaximumBlockCount(1000);layout->addWidget(details);
   for(const auto & event:events)list->addItem(QString::number(event.at("sequence").get<qulonglong>())+" "+QString::fromStdString(event.value("kind","")+" "+(event.value("kind","")=="operation_event"?event.at("payload").value("operation","")+" "+event.at("payload").value("state",""):"")));
   QTemporaryDir config_directory;QProcess player,rviz;QString buffer;int64_t start_ns=0;size_t event_cursor=0;uint64_t last_sequence=0;
   auto environment=QProcessEnvironment::systemEnvironment();const auto live_domain=environment.value("ROS_DOMAIN_ID","0");environment.insert("ROS_DOMAIN_ID",live_domain=="219"?"218":"219");environment.insert("ROS_LOCALHOST_ONLY","1");
   player.setProcessEnvironment(environment);rviz.setProcessEnvironment(environment);
   auto stop=[&]{for(auto process:{&player,&rviz})if(process->state()!=QProcess::NotRunning){process->terminate();if(!process->waitForFinished(1500)){process->kill();process->waitForFinished(1500);}}};
   unsigned restart_attempts=0;std::function<void()> restart;
   restart=[&]{
     if(player.state()!=QProcess::NotRunning||rviz.state()!=QProcess::NotRunning){
       seek->setEnabled(false);
       if(++restart_attempts>40){player.kill();rviz.kill();}else{player.terminate();rviz.terminate();}
       QTimer::singleShot(50,&window,[&]{restart();});return;
     }
     restart_attempts=0;seek->setEnabled(true);buffer.clear();event_cursor=0;last_sequence=0;
     const auto prefix="/astribot_replay_"+QUuid::createUuid().toString(QUuid::WithoutBraces).remove('-');
     QFile config(config_directory.path()+"/replay.rviz");if(!config.open(QIODevice::WriteOnly))throw std::runtime_error("Cannot write replay config");QTextStream out(&config);
     out<<"Panels:\n  - Class: rviz_common/Displays\n    Name: Displays\nVisualization Manager:\n  Class: ''\n  Global Options:\n    Fixed Frame: ";out<<QString::fromStdString(Json(frame->text().toStdString()).dump())<<"\n  Displays:\n";
     for(auto item:std::vector<std::pair<QString,QString>>{{"Map",QString::fromStdString(map_topic)},{"Path","/plan"},{"Odometry","/odom"}})
       out<<"    - Class: rviz_default_plugins/"<<item.first<<"\n      Name: "<<item.first<<"\n      Enabled: true\n      Topic:\n        Value: "<<prefix<<item.second<<"\n        Durability Policy: Transient Local\n";
     out<<"    - Class: rviz_default_plugins/TF\n      Name: TF\n      Enabled: true\n  Tools:\n    - Class: rviz_default_plugins/MoveCamera\n    - Class: rviz_default_plugins/Select\nWindow Geometry:\n  Width: 1000\n  Height: 750\n";out.flush();config.close();
     rviz.start("rviz2",{"-d",config.fileName(),"--ros-args","-r","/tf:="+prefix+"/tf","-r","/tf_static:="+prefix+"/tf_static","-r","/clock:="+prefix+"/clock","-p","use_sim_time:=true"});
     const auto executable=QString::fromStdString(ament_index_cpp::get_package_prefix("astribot_operator_station"))+"/lib/astribot_operator_station/replay_stream";
     player.start(executable,{QString::fromStdString(bag.string()),prefix,QString::number(seconds->value()),QString::number(rate->value())});
   };
   QObject::connect(&player,&QProcess::readyReadStandardOutput,&window,[&]{buffer+=player.readAllStandardOutput();int newline;
     while((newline=buffer.indexOf('\n'))>=0){auto line=buffer.left(newline);buffer.remove(0,newline+1);
       if(line.startsWith("META ")){auto j=Json::parse(line.mid(5).toStdString());start_ns=j.at("start_ns");seconds->setMaximum(j.at("duration_sec").get<double>());}
       if(line.startsWith("POSITION ")){int64_t stamp=line.mid(9).toLongLong();label->setText(QString("OFFLINE · 时间 %1 s · 重建/播放均只发布显示白名单；缺失参数标 unknown").arg((stamp-start_ns)/1e9));
         uint64_t seq=last_sequence;while(event_cursor<events.size()&&events[event_cursor].value("ros_ns",int64_t(-1))<=stamp){seq=events[event_cursor].at("sequence");++event_cursor;}
         if(seq&&seq!=last_sequence)details->setPlainText(QString::fromStdString(astribot_operator_station::parameters_at(events,seq).dump(2)));
         last_sequence=seq;
       }
       if(line=="EOF")label->setText("播放结束；RViz 保留最后画面。再次定位会重建缓存。");
     }
   });
   QObject::connect(&rviz,&QProcess::readyReadStandardOutput,&window,[&]{rviz.readAllStandardOutput();});
   QObject::connect(&rviz,&QProcess::readyReadStandardError,&window,[&]{auto error=rviz.readAllStandardError();if(error.contains("ERROR"))details->appendPlainText(QString::fromUtf8(error));});
   QObject::connect(&rviz,&QProcess::errorOccurred,&window,[&](QProcess::ProcessError){label->setText("3D 窗口异常："+rviz.errorString());});
   QObject::connect(&player,&QProcess::readyReadStandardError,&window,[&]{details->appendPlainText(QString::fromUtf8(player.readAllStandardError()));});
   QObject::connect(&player,&QProcess::errorOccurred,&window,[&](QProcess::ProcessError){label->setText("回放进程异常："+player.errorString());});
   QObject::connect(pause,&QPushButton::clicked,&window,[&]{player.write("{\"paused\":true}\n");});
   QObject::connect(play,&QPushButton::clicked,&window,[&]{player.write("{\"paused\":false}\n");});
   QObject::connect(rate,qOverload<double>(&QDoubleSpinBox::valueChanged),&window,[&](double value){player.write(QByteArray::fromStdString(Json({{"rate",value}}).dump()+"\n"));});
   QObject::connect(seek,&QPushButton::clicked,&window,[&]{restart();});
   QObject::connect(list,&QListWidget::currentRowChanged,&window,[&](int index){if(index<0)return;const auto & e=events.at(index);
     details->setPlainText(QString::fromStdString(e.dump(2)+"\n"+astribot_operator_station::parameters_at(events,e.at("sequence")).dump(2)));
     if(start_ns)seconds->setValue(std::max(0.,(e.value("ros_ns",start_ns)-start_ns)/1e9));
   });
   window.show();restart();int result=app.exec();stop();return result;
 }catch(const std::exception & e){std::cerr<<e.what()<<'\n';return 1;}
}
