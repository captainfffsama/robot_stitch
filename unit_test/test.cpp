/*
 * @Author: captainfffsama
 * @Date: 2022-12-21 20:37:03
 * @LastEditors: captainfffsama tuanzhangsama@outlook.com
 * @LastEditTime: 2023-01-03 10:11:28
 * @FilePath: \robot_stitch\unit_test\test.cpp
 * @Description:
 */
#include <algorithm>
#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"
#include "robotimagestitch.h"

int main() {
  cv::Mat src = cv::imread("D:\\temp\\1.jpg");

  RobotImageStitch imgStitch("localhost:52003", 0.25);
  cv::Mat result;
  imgStitch(src, result);
  cv::imwrite("D:\\temp\\r.jpg", result);
  return 0;
}
