#ifndef ROBOTIMAGESTITCH_H
#define ROBOTIMAGESTITCH_H

#include "robot_stitch_global.h"
#include "opencv2/opencv.hpp"

#include "alignrpc.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <limits>

class ROBOT_STITCH_EXPORT RobotImageStitch
{
public:
    RobotImageStitch(const std::string& host,float rRate=0.25);

    int operator()(const cv::Mat& img,cv::Mat& outImg) const;

private:
    std::shared_ptr<AlignRpc> aligner;
    float resizeFactor;
    cv::Mat rMInv,rM;

    void getImgPatch(const cv::Mat &srcImg, std::vector<cv::Mat> &imgPatchs, const cv::Vec3b &colorUpper = cv::Vec3b(77, 255, 255), const cv::Vec3b &colorLower = cv::Vec3b(35, 43, 36)) const;

};

namespace {
    bool isSameColor(const cv::Vec3b& betest,const cv::Vec3b& upper,const cv::Vec3b& lower);

    void cvPointTransByH(const std::vector<cv::Point2f>& srcPoint, const cv::Mat& mat, std::vector<cv::Point2f>& dstPoint);

    void resizeImg(const cv::Mat& input,cv::Mat& output,float xRate=0.25,float yRate=0.25);
}

#endif // ROBOTIMAGESTITCH_H
