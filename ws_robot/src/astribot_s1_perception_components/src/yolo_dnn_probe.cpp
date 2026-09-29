#include <chrono>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include "astribot_s1_perception_components/yolo_dnn.hpp"
int main(int argc,char ** argv) {
  if(argc!=5) {std::cerr<<"usage: yolo_dnn_probe model.onnx image labels.txt yolov5|yolov8\n";return 2;}
  try {
    cv::setNumThreads(2);
    const auto labels=astribot::vision::load_labels(argv[3]);
    astribot::vision::YoloDnn detector(argv[1],int(labels.size()),argv[4]);
    const auto frame=cv::imread(argv[2]);
    const auto start=std::chrono::steady_clock::now(); const auto boxes=detector.infer(frame);
    const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    std::cout<<"class,score,x,y,width,height\n";
    for(const auto & b:boxes) std::cout<<labels.at(size_t(b.class_id))<<","<<b.score<<","<<b.box.x<<","<<b.box.y<<","<<b.box.width<<","<<b.box.height<<"\n";
    std::cerr<<"inference_ms="<<ms<<" detections="<<boxes.size()<<"\n";return 0;
  } catch(const std::exception & e) {std::cerr<<e.what()<<"\n";return 1;}
}
