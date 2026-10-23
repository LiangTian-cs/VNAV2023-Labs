#include "sift_feature_tracker.h"
#include <opencv2/features2d.hpp>

SiftFeatureTracker::SiftFeatureTracker() : detector(cv::SIFT::create()) {}
void SiftFeatureTracker::detectKeypoints(const cv::Mat& im, std::vector<cv::KeyPoint>* kp) const {
  detector->detect(im, *kp);
}
void SiftFeatureTracker::describeKeypoints(const cv::Mat& im, std::vector<cv::KeyPoint>* kp, cv::Mat* desc) const {
  detector->compute(im, *kp, *desc);
}
void SiftFeatureTracker::matchDescriptors(const cv::Mat& a, const cv::Mat& b,
    std::vector<std::vector<cv::DMatch>>* matches, std::vector<cv::DMatch>* good) const {
  matches->clear(); good->clear();
  if (a.empty() || b.rows < 2) return;
  cv::FlannBasedMatcher matcher;
  matcher.knnMatch(a,b,*matches,2);
  for (const auto& m : *matches)
    if (m.size() >= 2 && m[0].distance < 0.8f*m[1].distance) good->push_back(m[0]);
}
