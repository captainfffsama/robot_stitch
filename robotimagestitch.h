#ifndef ROBOTIMAGESTITCH_H
#define ROBOTIMAGESTITCH_H

#include <math.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

#include "alignrpc.h"
#include "opencv2/opencv.hpp"
#include "robot_stitch_global.h"

class ROBOT_STITCH_EXPORT RobotImageStitch {
 public:
  explicit RobotImageStitch(
      const std::string& host, float rRate = 0.25,
      const cv::Vec3b& colorRangeUpper = cv::Vec3b(77, 255, 255),
      const cv::Vec3b& colorRangeLower = cv::Vec3b(35, 43, 36));

  int operator()(const cv::Mat& img, cv::Mat& outImg) const;

 private:
  std::shared_ptr<AlignRpc> aligner;
  const float resizeFactor;
  cv::Mat rMInv, rM;
  const cv::Vec3b colorUpper, colorLower;

  void getImgPatch(const cv::Mat& srcImg, std::vector<cv::Mat>& imgPatchs,
                   const cv::Vec3b& colorUpper = cv::Vec3b(77, 255, 255),
                   const cv::Vec3b& colorLower = cv::Vec3b(35, 43, 36)) const;
};

namespace {
bool isSameColor(const cv::Vec3b& betest, const cv::Vec3b& upper,
                 const cv::Vec3b& lower);

void cvPointTransByH(const std::vector<cv::Point2f>& srcPoint,
                     const cv::Mat& mat, std::vector<cv::Point2f>& dstPoint);

void resizeImg(const cv::Mat& input, cv::Mat& output, float xRate = 0.25,
               float yRate = 0.25);
}  // namespace

#endif  // ROBOTIMAGESTITCH_H
