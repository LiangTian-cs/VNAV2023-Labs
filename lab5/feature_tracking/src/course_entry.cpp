#include "sift_feature_tracker.h"
#include "lk_feature_tracker.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <memory>
#include <cstdlib>

int main(int argc,char** argv) {
  ros::init(argc,argv,"keypoint_trackers");
  ros::NodeHandle nh("~");
  cv::setNumThreads(1);cv::setRNGSeed(20261022);
  std::string out=nh.param<std::string>("output_dir","");
  if(out.empty()) {ROS_FATAL("output_dir required");return 1;}
  setenv("LAB5_OUTPUT_DIR",out.c_str(),1);
  std::string descriptor=nh.param<std::string>("descriptor","SIFT");
  std::unique_ptr<SiftFeatureTracker> sift;
  std::unique_ptr<LKFeatureTracker> lk;
  if(descriptor=="SIFT")sift.reset(new SiftFeatureTracker());
  else if(descriptor=="LK")lk.reset(new LKFeatureTracker());
  else {ROS_FATAL("Only implemented SIFT/LK supported; optional templates not enabled");return 1;}
  if(nh.param<int>("mode",0)==0) {
    if(!sift) return 1;
    auto a=cv::imread(nh.param<std::string>("img1_path",""),0);
    auto b=cv::imread(nh.param<std::string>("img2_path",""),0);
    if(a.empty()||b.empty())return 1;
    sift->trackFeatures(a,b);sift->printStats();return 0;
  }
  cv::Mat previous;ros::Time last;unsigned frames=0;
  std::ofstream events(out+"/events.csv"),metrics(out+"/sift_frames.csv");
  if(!events||!metrics)return 1;
  events<<"stamp_ns,event,frames\n";
  metrics<<"stamp_ns,kp1,kp2,raw_matches,good_matches,inliers,inlier_ratio,descriptor_dim,geometry_valid\n";
  auto callback=[&](const sensor_msgs::ImageConstPtr& msg) {
    if(!last.isZero()&&msg->header.stamp<=last) {
      events<<msg->header.stamp.toNSec()<<",REJECT_NONMONOTONIC,"<<frames<<std::endl;return;
    }
    try {
      auto im=cv_bridge::toCvShare(msg,"mono8")->image;
      if(lk)lk->trackFeatures(im);
      else if(!previous.empty()) {
        sift->trackFeatures(previous,im);
        std::ifstream f(out+"/SIFT_PAIR.csv");std::string h,row;
        std::getline(f,h);std::getline(f,row);metrics<<msg->header.stamp.toNSec()<<","<<row<<std::endl;
      }
      im.copyTo(previous);last=msg->header.stamp;++frames;
      events<<last.toNSec()<<",PROCESSED,"<<frames<<std::endl;
    } catch(const std::exception& e) {
      ROS_ERROR_STREAM(e.what());events<<msg->header.stamp.toNSec()<<",REJECT_IMAGE,"<<frames<<std::endl;
    }
  };
  auto sub=nh.subscribe<sensor_msgs::Image>("/images_topic",100,callback);
  ros::spin();if(sift)sift->printStats();return 0;
}
