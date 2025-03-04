/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2021; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度最新板卡PPNCx), The specific details consult the
 *professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file camera_display.cpp
 * @author Leo (liaotengjun@saishukeji.com)
 * @brief 相机采图测试Demo
 * @version 0.1
 * @date 2022-02-23
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "../include/common.hpp"
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
  std::string windowName = "frame";
  cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
  cv::resizeWindow(windowName, 640, 480);     // 分辨率
  cv::moveWindow(windowName, 300, 300);       // 布局位置

  /*注意：
    使用 0 和 /dev/video0 的分辨率不同：
      0           : opencv 内部的采集，可能是基于 V4L2, 分辨率：1280 * 960
      /dev/video0 : 基于Gstreamer ， 分辨率：640 * 480
  */
  VideoCapture capture("/dev/video0");
  if (!capture.isOpened()) {
    std::cout << "can not open video device " << std::endl;
    return 1;
  }
  capture.set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);

  double rate = capture.get(CAP_PROP_FPS);
  double width = capture.get(CAP_PROP_FRAME_WIDTH);
  double height = capture.get(CAP_PROP_FRAME_HEIGHT);
  std::cout << "Camera Param: frame rate = " << rate << " width = " << width
            << " height = " << height << std::endl;

  while (1) {
    Mat frame;
    if (!capture.read(frame)) {
      std::cout << "no video frame" << std::endl;
      continue;
    }

    // 绘制田字格：基准线
    uint16_t rows = ROWSIMAGE / 30; // 8
    uint16_t cols = COLSIMAGE / 32; // 10

    for (size_t i = 1; i < rows; i++) {
      line(frame, Point(0, 30 * i), Point(frame.cols - 1, 30 * i),
           Scalar(211, 211, 211), 1);
    }
    for (size_t i = 1; i < cols; i++) {
      if (i == cols / 2)
        line(frame, Point(32 * i, 0), Point(32 * i, frame.rows - 1),
             Scalar(0, 0, 255), 1);
      else
        line(frame, Point(32 * i, 0), Point(32 * i, frame.rows - 1),
             Scalar(211, 211, 211), 1);
    }

    imshow("frame", frame);
    waitKey(10);
  }
  capture.release();
}