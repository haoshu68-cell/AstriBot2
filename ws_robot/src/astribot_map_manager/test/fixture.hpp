#pragma once
#include <astribot_s1_autonomy/session_archive.hpp>
inline void makeMap(const std::filesystem::path & dir){
 namespace fs=std::filesystem;fs::create_directories(dir/"kf");
 std::ofstream(dir/(dir.filename().string()+".yaml"))<<"image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n";
 std::ofstream image(dir/"map.pgm",std::ios::binary);image<<"P5\n100 100\n255\n";const std::string pixels(10000,static_cast<char>(255));image.write(pixels.data(),pixels.size());image.close();
 std::ofstream pose(dir/"alidarState.txt");for(int i=0;i<26;++i)pose<<(i==7?1:0)<<' ';pose<<'\n';pose.close();
 std::ofstream(dir/"kf/0.pcd")<<std::string(150,'x');
 auto manifest=astribot_s1_autonomy::inspectSession(dir);manifest["exploration_outcome"]="COMPLETED";
 astribot_s1_autonomy::commitSession(dir,manifest,1);
}
