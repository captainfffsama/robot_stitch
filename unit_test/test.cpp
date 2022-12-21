/*
 * @Author: captainfffsama
 * @Date: 2022-12-21 20:37:03
 * @LastEditors: captainfffsama tuanzhangsama@outlook.com
 * @LastEditTime: 2022-12-22 00:14:41
 * @FilePath: \robot_stitch\unit_test\test.cpp
 * @Description:
 */
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <algorithm>
#include "robotimagestitch.h"

// bool isSameColor(const cv::Vec3b &betest, const cv::Vec3b &upper, const cv::Vec3b &lower)
// {
//     return (betest[0] < upper[0]) && (betest[0] > lower[0]) && (betest[1] < upper[1]) && (betest[1] > lower[1]) && (betest[2] < upper[2]) && (betest[2] > lower[2]);
// }

// void getImgPatch(const cv::Mat &srcImg, std::vector<cv::Mat> &imgPatchs, const cv::Vec3b &colorUpper = cv::Vec3b(77, 255, 255), const cv::Vec3b &colorLower = cv::Vec3b(35, 43, 36))
// {
//     cv::Mat hsvSrc;
//     cv::cvtColor(srcImg, hsvSrc, cv::COLOR_BGR2HSV);

//     std::vector<int> colCount(hsvSrc.cols, 0);

//     hsvSrc.forEach<cv::Vec3b>([&colCount, &colorUpper, &colorLower](cv::Vec3b &pixel, const int position[]) -> void
//                               {
//         if(isSameColor(pixel,colorUpper,colorLower))
//         {
//             colCount[position[1]]++;
//         } });
//     auto coliter = std::max_element(colCount.begin(), colCount.end(), [](int i, int j) -> bool
//                                     { return i < j; });

//     int colIndex = coliter - colCount.begin();

//     std::vector<int> rowCount(hsvSrc.rows, 0);
//     hsvSrc(cv::Range(0, hsvSrc.rows), cv::Range(0, colIndex)).forEach<cv::Vec3b>([&rowCount, &colorUpper, &colorLower](cv::Vec3b &pixel, const int position[]) -> void
//                                                                                  {
//         if(isSameColor(pixel,colorUpper,colorLower))
//         {
//             rowCount[position[0]]++;
//         } });

//     auto rowiter = std::max_element(rowCount.begin(), rowCount.end(), [](int i, int j) -> bool
//                                     { return i < j; });
//     int rowIndex = rowiter - rowCount.begin();

//     imgPatchs.push_back(srcImg(cv::Range(0, rowIndex - 5), cv::Range(0, colIndex - 5)).clone());
//     imgPatchs.push_back(srcImg(cv::Range(rowIndex + 5, hsvSrc.rows), cv::Range(0, colIndex - 5)).clone());
// }

int main()
{
    cv::Mat src = cv::imread("D:\\temp\\1.jpg");

    RobotImageStitch imgStitch("127.0.0.1:52003");
    cv::Mat result;
    imgStitch(src,result);
    cv::imwrite("D:\\temp\\r.jpg", result);
    return 0;
}
