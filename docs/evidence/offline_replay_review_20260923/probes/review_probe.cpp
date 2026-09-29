#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <thread>
#include <spawn.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
extern char **environ;
using J=nlohmann::json;namespace fs=std::filesystem;using Clock=std::chrono::steady_clock;
nav_msgs::msg::OccupancyGrid grid(int value){nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.info.width=m.info.height=1;m.info.resolution=.1;m.info.origin.orientation.w=1.;m.data={static_cast<int8_t>(value)};return m;}
int main(int argc,char**argv){if(argc!=3)return 2;fs::path root=argv[2];fs::create_directories(root);
 {rosbag2_cpp::Writer w;w.open((root/"seek"/"bag").string());for(int i=0;i<1200;++i)w.write(grid(3),"/map",rclcpp::Time(100000000000LL+i*2000000LL));w.write(grid(120),"/map",rclcpp::Time(102400000000LL));w.write(grid(121),"/map",rclcpp::Time(102600000000LL));}
 {std::ofstream f(root/"seek"/"events.jsonl");f<<J({{"sequence",1},{"kind","parameter_snapshot"},{"ros_ns",101000000000LL},{"node","/example"},{"values",{{"speed",{{"type",3},{"value",.3},{"available",true}}}}}}).dump()<<'\n';f<<J({{"sequence",2},{"kind","parameter_event"},{"ros_ns",105000000000LL},{"node","/example"},{"values",{{"speed",{{"type",3},{"value",.1},{"available",true}}}}}}).dump()<<'\n';}
 {rosbag2_cpp::Writer w;w.open((root/"rollback"/"bag").string());for(int sec:{20,21,1,2})w.write(grid(sec),"/map",rclcpp::Time(int64_t(sec)*1000000000));}
 J report;report["rollback_written_seconds"]={20,21,1,2};report["rollback_reader_seconds"]=J::array();{rosbag2_cpp::Reader reader;reader.open((root/"rollback"/"bag").string());while(reader.has_next())report["rollback_reader_seconds"].push_back(reader.read_next()->time_stamp/1000000000);}
 rclcpp::init(0,nullptr);auto node=std::make_shared<rclcpp::Node>("offline_replay_review_probe");
 for(const auto &trial:std::vector<std::pair<std::string,std::string>>{{"normal","0"},{"seek","2.4"}}){
  const auto prefix="/astribot_replay_review_"+trial.first+"_"+std::to_string(getpid());Clock::time_point target{},after{};
  auto sub=node->create_subscription<nav_msgs::msg::OccupancyGrid>(prefix+"/map",rclcpp::QoS(100).reliable().transient_local(),[&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){if(m->data.at(0)==120)target=Clock::now();if(m->data.at(0)==121)after=Clock::now();});
  std::vector<std::string> args={argv[1],(root/"seek"/"bag").string(),prefix,trial.second,"1"};std::vector<char*>av;for(auto&s:args)av.push_back(s.data());av.push_back(nullptr);
  posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);auto log=(root/(trial.first+"_player.log")).string();posix_spawn_file_actions_addopen(&actions,1,log.c_str(),O_WRONLY|O_CREAT|O_TRUNC,0600);posix_spawn_file_actions_adddup2(&actions,1,2);posix_spawn_file_actions_addopen(&actions,0,"/dev/null",O_RDONLY,0);
  pid_t child=-1;if(posix_spawn(&child,av[0],&actions,nullptr,av.data(),environ))throw std::runtime_error("spawn failed");posix_spawn_file_actions_destroy(&actions);
  auto end=Clock::now()+std::chrono::seconds(6);while(Clock::now()<end&&after.time_since_epoch().count()==0){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  kill(child,SIGTERM);int code;waitpid(child,&code,0);report[trial.first]={{"target_received",target.time_since_epoch().count()!=0},{"next_received",after.time_since_epoch().count()!=0},{"expected_gap_ms",200},{"observed_gap_ms",std::chrono::duration<double,std::milli>(after-target).count()},{"owned_child_pid",child},{"child_exit",WIFEXITED(code)?WEXITSTATUS(code):-1}};
 }
 rclcpp::shutdown();std::ofstream(root/"probe_results.json")<<report.dump(2)<<'\n';std::cout<<report.dump(2)<<'\n';
}
