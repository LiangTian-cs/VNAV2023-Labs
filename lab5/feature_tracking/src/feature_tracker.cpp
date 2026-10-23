#include "feature_tracker.h"
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>
#include <numeric>
#include <cstdlib>
#include <stdexcept>

static std::string output() {
  const char* p=std::getenv("LAB5_OUTPUT_DIR");
  if (!p) throw std::runtime_error("LAB5_OUTPUT_DIR must name an existing output directory");
  return std::string(p)+"/";
}
static unsigned processed_pairs=0;
static void save(const std::string& name,const cv::Mat& im) {
  if(processed_pairs>1 && processed_pairs%100!=0)return;
  if (!cv::imwrite(output()+name,im)) throw std::runtime_error("imwrite failed: "+name);
}
void FeatureTracker::trackFeatures(const cv::Mat& a,const cv::Mat& b,
    std::pair<std::vector<cv::KeyPoint>,std::vector<cv::KeyPoint>>* result) {
  if (a.empty() || b.empty()) throw std::runtime_error("Empty input image");
  ++processed_pairs;
  std::vector<cv::KeyPoint> k1,k2;
  cv::Mat d1,d2,draw;
  detectKeypoints(a,&k1); detectKeypoints(b,&k2);
  cv::drawKeypoints(a,k1,draw,cv::Scalar(0,255,0),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); save("keypoints_1.png",draw);
  cv::drawKeypoints(b,k2,draw,cv::Scalar(0,255,0),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); save("keypoints_2.png",draw);
  describeKeypoints(a,&k1,&d1); describeKeypoints(b,&k2,&d2);
  std::vector<std::vector<cv::DMatch>> knn;
  std::vector<cv::DMatch> good,raw;
  matchDescriptors(d1,d2,&knn,&good);
  for (const auto& m:knn) if (!m.empty()) raw.push_back(m[0]);
  cv::drawMatches(a,k1,b,k2,raw,draw); save("matches_before_ratio.png",draw);
  cv::drawMatches(a,k1,b,k2,good,draw); save("matches_after_ratio.png",draw);
  std::vector<cv::KeyPoint> g1,g2;
  for (const auto& m:good) {g1.push_back(k1.at(m.queryIdx));g2.push_back(k2.at(m.trainIdx));}
  std::vector<uchar> mask;
  if (good.size()>=8) inlierMaskComputation(g1,g2,&mask);
  const bool geometry_valid=mask.size()==good.size() && good.size()>=8;
  if (!geometry_valid) mask.assign(good.size(),0);
  unsigned int n=std::accumulate(mask.begin(),mask.end(),0u);
  std::vector<char> cm(mask.begin(),mask.end());
  cv::drawMatches(a,k1,b,k2,good,draw,cv::Scalar(0,0,255),cv::Scalar::all(-1),{},cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  cv::drawMatches(a,k1,b,k2,good,draw,cv::Scalar(0,255,0),cv::Scalar::all(-1),cm,
      cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS|cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
  save("inliers_green_outliers_red.png",draw);
  if (result) {
    result->first.clear(); result->second.clear();
    for(size_t i=0;i<mask.size();++i) if(mask[i]) {result->first.push_back(g1[i]);result->second.push_back(g2[i]);}
  }
  const float old=num_samples_, count=old+1;
  avg_num_keypoints_img1_=(avg_num_keypoints_img1_*old+k1.size())/count;
  avg_num_keypoints_img2_=(avg_num_keypoints_img2_*old+k2.size())/count;
  avg_num_matches_=(avg_num_matches_*old+raw.size())/count;
  avg_num_good_matches_=(avg_num_good_matches_*old+good.size())/count;
  avg_num_inliers_=(avg_num_inliers_*old+n)/count;
  avg_inlier_ratio_=(avg_inlier_ratio_*old+(good.empty()?0.0:double(n)/good.size()))/count;
  ++num_samples_;
  std::ofstream f(output()+"SIFT_PAIR.csv");
  f<<"kp1,kp2,raw_matches,good_matches,inliers,inlier_ratio,descriptor_dim,geometry_valid\n";
  f<<k1.size()<<","<<k2.size()<<","<<raw.size()<<","<<good.size()<<","<<n<<","<<(good.empty()?0.0:double(n)/good.size())<<","<<d1.cols<<","<<geometry_valid<<"\n";
}
void FeatureTracker::inlierMaskComputation(const std::vector<cv::KeyPoint>& a,
    const std::vector<cv::KeyPoint>& b,std::vector<uchar>* mask) const {
  // Same judge as the course: F-matrix RANSAC, 3 px, confidence 0.99.
  std::vector<cv::Point2f> p,q;
  for(size_t i=0;i<a.size();++i){p.push_back(a[i].pt);q.push_back(b.at(i).pt);}
  cv::findFundamentalMat(p,q,cv::FM_RANSAC,3.0,0.99,*mask);
}
void FeatureTracker::drawMatches(const cv::Mat& a,const cv::Mat& b,
    const std::vector<cv::KeyPoint>& p,const std::vector<cv::KeyPoint>& q,
    const std::vector<std::vector<cv::DMatch>>& matches) {
  cv::Mat out;cv::drawMatches(a,p,b,q,matches,out);save("matches.png",out);
}
void FeatureTracker::printStats() const {
  std::cout<<"samples="<<num_samples_<<" kp="<<avg_num_keypoints_img1_<<","<<avg_num_keypoints_img2_
      <<" raw="<<avg_num_matches_<<" good="<<avg_num_good_matches_<<" inliers="<<avg_num_inliers_
      <<" ratio="<<avg_inlier_ratio_<<std::endl;
}
