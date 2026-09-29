#pragma once
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <stdexcept>
#include <vector>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace astribot::vision {
struct Letterbox {
  cv::Mat image;
  cv::Size original;
  double sx, sy;
  int left, top;
};
struct YoloBox { cv::Rect box; float score; int class_id; };
inline Letterbox letterbox(const cv::Mat & bgr,int size) {
  if(bgr.empty() || bgr.type()!=CV_8UC3 || size<32 || size>2048 || bgr.cols>8192 || bgr.rows>8192)
    throw std::invalid_argument("invalid BGR image or network size");
  const double scale=std::min(double(size)/bgr.cols,double(size)/bgr.rows);
  const int w=std::max(1,int(std::round(bgr.cols*scale))),h=std::max(1,int(std::round(bgr.rows*scale)));
  Letterbox r; r.original=bgr.size(); r.sx=double(w)/bgr.cols; r.sy=double(h)/bgr.rows;
  r.left=(size-w)/2; r.top=(size-h)/2;
  cv::Mat resized; cv::resize(bgr,resized,cv::Size(w,h));
  cv::copyMakeBorder(resized,r.image,r.top,size-h-r.top,r.left,size-w-r.left,cv::BORDER_CONSTANT,cv::Scalar(114,114,114));
  return r;
}
// Only decoded float32 detection graphs: v5 [1,N,5+C], v8 [1,4+C,N].
// Segmentation prototypes, end-to-end NMS graphs and other layouts are rejected.
inline std::vector<YoloBox> decode_yolo(const cv::Mat & tensor,const Letterbox & lb,int classes,
  const std::string & layout,float confidence,float iou,int maximum)
{
  if(tensor.type()!=CV_32F || tensor.dims!=3 || tensor.size[0]!=1 || classes<1 || classes>10000 ||
     (layout!="yolov5" && layout!="yolov8") || !std::isfinite(confidence) || confidence<0 || confidence>1 ||
     !std::isfinite(iou) || iou<=0 || iou>1 || maximum<1 || maximum>1000)
    throw std::invalid_argument("unsupported YOLO tensor or thresholds");
  const bool v5=layout=="yolov5";
  if((v5 ? tensor.size[2] : tensor.size[1])!=classes+(v5?5:4))
    throw std::invalid_argument("YOLO output class/layout mismatch (segmentation not supported)");
  const int rows=v5?tensor.size[1]:tensor.size[2];
  if(rows<1 || rows>100000) throw std::invalid_argument("YOLO output exceeds candidate budget");
  const cv::Mat contiguous=tensor.isContinuous()?tensor:tensor.clone();
  const float * p=contiguous.ptr<float>();
  auto at=[&](int n,int c){return v5?p[n*(classes+5)+c]:p[c*rows+n];};
  std::vector<YoloBox> boxes;
  for(int n=0;n<rows;++n) {
    const float objectness=v5?at(n,4):1.f;
    if(!std::isfinite(objectness) || objectness<confidence || objectness>1) continue;
    int id=-1; float score=0;
    for(int c=0;c<classes;++c) {
      const float probability=at(n,c+(v5?5:4));
      if(std::isfinite(probability) && probability>=0 && probability<=1 && probability*objectness>score) {
        id=c;score=probability*objectness;
      }
    }
    if(id<0 || score<confidence) continue;
    const double cx=at(n,0),cy=at(n,1),w=at(n,2),h=at(n,3);
    if(!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) || !std::isfinite(h) || w<=0 || h<=0) continue;
    const auto clip=[](double v,int limit){return std::clamp(v,0.,double(limit));};
    const int x0=int(std::floor(clip((cx-w/2-lb.left)/lb.sx,lb.original.width)));
    const int y0=int(std::floor(clip((cy-h/2-lb.top)/lb.sy,lb.original.height)));
    const int x1=int(std::ceil(clip((cx+w/2-lb.left)/lb.sx,lb.original.width)));
    const int y1=int(std::ceil(clip((cy+h/2-lb.top)/lb.sy,lb.original.height)));
    if(x1>x0 && y1>y0) boxes.push_back({cv::Rect(x0,y0,x1-x0,y1-y0),score,id});
  }
  std::stable_sort(boxes.begin(),boxes.end(),[](const auto & a,const auto & b){return a.score>b.score;});
  if(boxes.size()>3000) boxes.resize(3000);
  std::map<int,std::vector<size_t>> groups;
  for(size_t i=0;i<boxes.size();++i) groups[boxes[i].class_id].push_back(i);
  std::vector<YoloBox> out;
  for(const auto & entry:groups) {
    std::vector<cv::Rect> rects;std::vector<float> scores;
    for(const auto index:entry.second) {rects.push_back(boxes[index].box);scores.push_back(boxes[index].score);}
    std::vector<int> kept;cv::dnn::NMSBoxes(rects,scores,confidence,iou,kept);
    for(const auto index:kept) out.push_back(boxes[entry.second[size_t(index)]]);
  }
  std::stable_sort(out.begin(),out.end(),[](const auto & a,const auto & b){return a.score>b.score;});
  if(out.size()>size_t(maximum)) out.resize(size_t(maximum));
  return out;
}
inline std::vector<std::string> load_labels(const std::string & path) {
  std::ifstream stream(path); if(!stream) throw std::invalid_argument("label file unavailable");
  std::vector<std::string> labels;std::string line;
  while(std::getline(stream,line)) {
    if(!line.empty() && line.back()=='\r') line.pop_back();
    if(line.empty()) throw std::invalid_argument("blank class label");
    labels.push_back(line);
  }
  if(labels.empty()) throw std::invalid_argument("no class labels");
  return labels;
}
class YoloDnn {
public:
  YoloDnn(const std::string & path,int classes,const std::string & layout,int size=640)
    :classes_(classes),size_(size),layout_(layout) {
    if(path.empty() || classes<1 || size<32 || size>2048 || (layout!="yolov5" && layout!="yolov8"))
      throw std::invalid_argument("model path, layout, class count and size required");
    net_=cv::dnn::readNetFromONNX(path);
    if(net_.empty()) throw std::invalid_argument("ONNX graph unavailable");
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV); net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  }
  std::vector<YoloBox> infer(const cv::Mat & bgr,float confidence=.25,float iou=.45,int maximum=100) {
    const auto lb=letterbox(bgr,size_);
    net_.setInput(cv::dnn::blobFromImage(lb.image,1./255.,lb.image.size(),cv::Scalar(),true,false));
    std::vector<cv::Mat> outputs; net_.forward(outputs,net_.getUnconnectedOutLayersNames());
    if(outputs.size()!=1) throw std::invalid_argument("single decoded detection output required");
    return decode_yolo(outputs.front(),lb,classes_,layout_,confidence,iou,maximum);
  }
private:
  cv::dnn::Net net_; int classes_,size_;std::string layout_;
};
} // namespace astribot::vision
