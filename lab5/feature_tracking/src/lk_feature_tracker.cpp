#include "lk_feature_tracker.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <numeric>
#include <stdexcept>

LKFeatureTracker::LKFeatureTracker() {}
LKFeatureTracker::~LKFeatureTracker() {}
void LKFeatureTracker::trackFeatures(const cv::Mat& frame) {
  cv::Mat gray;
  if(frame.channels()==1) gray=frame.clone();else cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
  if(gray.empty()) throw std::runtime_error("Empty LK frame");
  if(prev_frame_.empty()) {gray.copyTo(prev_frame_);return;}
  if(prev_corners_.size()<50)
    cv::goodFeaturesToTrack(prev_frame_,prev_corners_,500,0.01,10.0,cv::noArray(),3,true,0.04);
  std::vector<cv::Point2f> next,a,b;
  std::vector<uchar> status;
  std::vector<float> err;
  if(!prev_corners_.empty()) cv::calcOpticalFlowPyrLK(prev_frame_,gray,prev_corners_,next,status,err,
      cv::Size(21,21),3,cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.01));
  // Forward/backward consistency rejects status-valid but unreliable tracks.
  std::vector<cv::Point2f> back;
  std::vector<uchar> back_status;
  std::vector<float> back_err;
  if(!next.empty())cv::calcOpticalFlowPyrLK(gray,prev_frame_,next,back,back_status,back_err,
      cv::Size(21,21),3,cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.01));
  for(size_t i=0;i<status.size();++i)
    if(status[i] && back_status[i] && cv::norm(back[i]-prev_corners_[i])<=1.0 &&
       std::isfinite(next[i].x) && std::isfinite(next[i].y) &&
       next[i].x>=0 && next[i].y>=0 && next[i].x<gray.cols && next[i].y<gray.rows) {
      a.push_back(prev_corners_[i]);b.push_back(next[i]);
    }
  std::vector<uchar> mask;
  if(a.size()>=8) inlierMaskComputation(a,b,&mask);
  bool valid=mask.size()==a.size() && a.size()>=8;
  if(!valid) mask.assign(a.size(),0);
  unsigned n=std::accumulate(mask.begin(),mask.end(),0u);
  const float count=num_samples_+1,old=num_samples_;
  avg_num_keypoints_img1_=(avg_num_keypoints_img1_*old+prev_corners_.size())/count;
  avg_num_keypoints_img2_=(avg_num_keypoints_img2_*old+b.size())/count;
  avg_num_matches_=(avg_num_matches_*old+a.size())/count;
  avg_num_inliers_=(avg_num_inliers_*old+n)/count;
  avg_inlier_ratio_=(avg_inlier_ratio_*old+(a.empty()?0.0:double(n)/a.size()))/count;
  ++num_samples_;
  const char* out=std::getenv("LAB5_OUTPUT_DIR");
  if(!out) throw std::runtime_error("LAB5_OUTPUT_DIR required");
  const std::string root=std::string(out)+"/";
  std::ofstream f(root+"LK.csv",std::ios::app);
  if(num_samples_==1)f<<"sample,kp_prev,tracked,inliers,ratio,geometry_valid\n";
  f<<num_samples_<<","<<prev_corners_.size()<<","<<a.size()<<","<<n<<","<<(a.empty()?0.0:double(n)/a.size())<<","<<valid<<"\n";
  if(num_samples_==1 || num_samples_%30==0)show(frame,a,b);
  std::ofstream pts(root+"LK_last_points.csv");pts<<"x0,y0,x1,y1\n";
  for(size_t i=0;i<a.size();++i)pts<<a[i].x<<","<<a[i].y<<","<<b[i].x<<","<<b[i].y<<"\n";
  prev_corners_=b;gray.copyTo(prev_frame_);
}
void LKFeatureTracker::show(const cv::Mat& frame,std::vector<cv::Point2f>& a,std::vector<cv::Point2f>& b) {
  cv::Mat out;
  if(frame.channels()==1)cv::cvtColor(frame,out,cv::COLOR_GRAY2BGR);else out=frame.clone();
  for(size_t i=0;i<a.size();++i){cv::line(out,a[i],b[i],cv::Scalar(0,255,0));cv::circle(out,b[i],2,cv::Scalar(0,0,255),-1);}
  if(!cv::imwrite(std::string(std::getenv("LAB5_OUTPUT_DIR"))+"/LK_tracks_"+std::to_string(num_samples_)+".png",out))
    throw std::runtime_error("LK image write failed");
}
void LKFeatureTracker::inlierMaskComputation(const std::vector<cv::Point2f>& a,
    const std::vector<cv::Point2f>& b,std::vector<uchar>* mask)const {
  try {
    cv::Mat F=cv::findFundamentalMat(a,b,cv::FM_RANSAC,3.0,0.99,*mask);
    if(F.empty() || F.rows!=3 || F.cols!=3 || !cv::checkRange(F))mask->clear();
  } catch(const cv::Exception& e) {
    // A failed geometric model does not invalidate the image or valid LK tracks.
    // Preserve the exact correspondences for independent reproduction; no fallback judge.
    const std::string root=std::string(std::getenv("LAB5_OUTPUT_DIR"))+"/";
    std::ofstream log(root+"F_FAILURES.log",std::ios::app);
    log<<"sample="<<num_samples_+1<<" points="<<a.size()<<" "<<e.what()<<std::endl;
    std::ofstream pts(root+"F_failure_"+std::to_string(num_samples_+1)+".csv");
    pts.precision(9);pts<<"x0,y0,x1,y1\n";
    for(size_t i=0;i<a.size();++i)pts<<a[i].x<<","<<a[i].y<<","<<b[i].x<<","<<b[i].y<<"\n";
    mask->clear();
  }
}
