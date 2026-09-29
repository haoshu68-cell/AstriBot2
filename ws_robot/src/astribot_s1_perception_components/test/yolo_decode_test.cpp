#include <gtest/gtest.h>
#include "astribot_s1_perception_components/yolo_dnn.hpp"
using namespace astribot::vision;
TEST(Yolo, LetterboxRetainsAspectAndPadding) {
 auto lb=letterbox(cv::Mat::zeros(100,200,CV_8UC3),640);
 EXPECT_EQ(lb.left,0); EXPECT_EQ(lb.top,160); EXPECT_DOUBLE_EQ(lb.sx,3.2); EXPECT_DOUBLE_EQ(lb.sy,3.2);
 EXPECT_EQ(lb.image.at<cv::Vec3b>(0,0)[0],114);
}
TEST(Yolo, V5BoxRestoredWithObjectness) {
 auto lb=letterbox(cv::Mat::zeros(100,200,CV_8UC3),640);
 int shape[]={1,1,7}; cv::Mat tensor(3,shape,CV_32F,cv::Scalar(0));
 const float values[]={320,320,320,160,.8,.9,.1}; std::copy(values,values+7,tensor.ptr<float>());
 auto result=decode_yolo(tensor,lb,2,"yolov5",.25,.45,20);
 ASSERT_EQ(result.size(),1u);EXPECT_EQ(result[0].box,cv::Rect(50,25,100,50)); EXPECT_NEAR(result[0].score,.72,1e-6);
}
TEST(Yolo, V8ChannelMajorNoObjectness) {
 auto lb=letterbox(cv::Mat::zeros(640,640,CV_8UC3),640);
 int shape[]={1,6,1};cv::Mat tensor(3,shape,CV_32F,cv::Scalar(0));
 const float values[]={320,320,100,100,.1,.9};std::copy(values,values+6,tensor.ptr<float>());
 auto result=decode_yolo(tensor,lb,2,"yolov8",.25,.45,20);
 ASSERT_EQ(result.size(),1u);EXPECT_EQ(result[0].class_id,1);EXPECT_NEAR(result[0].score,.9,1e-6);
}
TEST(Yolo, NmsPreservesDifferentClasses) {
 auto lb=letterbox(cv::Mat::zeros(640,640,CV_8UC3),640);
 int shape[]={1,3,7};cv::Mat tensor(3,shape,CV_32F,cv::Scalar(0));
 const float v[]={320,320,100,100,1,.9,.1,320,320,100,100,1,.8,.1,320,320,100,100,1,.1,.85};
 std::copy(v,v+21,tensor.ptr<float>());
 EXPECT_EQ(decode_yolo(tensor,lb,2,"yolov5",.25,.45,20).size(),2u);
}
TEST(Yolo, RejectsUnexpectedSegmentationOrClassShape) {
 auto lb=letterbox(cv::Mat::zeros(100,200,CV_8UC3),640);
 int shape[]={1,116,8400}; cv::Mat tensor(3,shape,CV_32F,cv::Scalar(0));
 EXPECT_THROW(decode_yolo(tensor,lb,80,"yolov8",.25,.45,20),std::invalid_argument);
}
TEST(Yolo, RejectsNonfiniteOrOffImageBox) {
 auto lb=letterbox(cv::Mat::zeros(640,640,CV_8UC3),640);
 int shape[]={1,2,6};cv::Mat t(3,shape,CV_32F,cv::Scalar(0));
 const float v[]={NAN,320,100,100,1,.9,1e30f,320,100,100,1,.9};std::copy(v,v+12,t.ptr<float>());
 EXPECT_TRUE(decode_yolo(t,lb,1,"yolov5",.25,.45,20).empty());
}
