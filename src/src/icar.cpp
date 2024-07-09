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
 * @file icar.cpp
 * @author Leo (liaotengjun@saishukeji.com)
 * @brief 智能汽车-完全模型组-顶层框架（TOP）
 * @version 0.1
 * @date 2022-12-12
 * @note 欢迎各位完全模型组的小伙伴，共同构建技术框架
 * @copyright Copyright (c) 2023
 *
 */
#include "../include/common.hpp"                //公共类方法文件
#include "../include/detection.hpp"             //PPNC目标检测
#include "../include/uart.hpp"                  //串口通信驱动
#include "controlcenter_cal.cpp"                //控制中心计算类
#include "detection/bridge_detection.cpp"       //桥梁AI检测与路径规划类
#include "detection/depot_detection.cpp"        //维修厂AI检测
#include "detection/farmland_detection.cpp"     //农田区域AI检测
#include "detection/granary_detection.cpp"      //粮仓AI检测
#include "detection/slowzone_detection.cpp"     //慢行区AI检测与路径规划类
#include "image_preprocess.cpp"                 //图像预处理类
#include "motion_controller.cpp"                //智能车运动控制类
#include "recognition/cross_recognition.cpp"    //十字道路识别与路径规划类
#include "recognition/freezone_recognition.cpp" //泛行区识别类
#include "recognition/garage_recognition.cpp"   //车库及斑马线识别类
#include "recognition/ring_recognition.cpp"     //环岛道路识别与路径规划类
#include "recognition/track_recognition.cpp"    //赛道识别基础类
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>
#include <cstdio>
using namespace std;
using namespace cv;

void callbackSignal(int signum);
void displayWindowInit(void);
void slowDownEnable(void);
bool slowDown = false;                    // 特殊区域减速标志
uint16_t counterSlowDown = 0;             // 减速计数器
std::shared_ptr<Driver> driver = nullptr; // 初始化串口驱动

enum RoadType
{
  BaseHandle = 0, // 基础赛道处理
  RingHandle,     // 环岛赛道处理
  CrossHandle,    // 十字道路处理
  FreezoneHandle, // 泛行区处理
  GarageHandle,   // 车库处理
  GranaryHandle,  // 粮仓处理
  DepotHandle,    // 修车厂处理
  BridgeHandle,   // 坡道(桥)处理
  SlowzoneHandle, // 慢行区（动物出没）处理
  FarmlandHandle, // 农田区域处理
};

int main(int argc, char const *argv[])
{
  // USB转串口的设备名为 / dev/ttyUSB0
  char ttyUSB[100];
  ttyUSB[0] = 0;
  std::string cmd = "ls /dev | grep ttyUSB";
  while(strlen(ttyUSB) == 0){
      executeCMD(cmd.c_str(), ttyUSB);
  }
  cout << ttyUSB << std::endl;
  ttyUSB[strlen(ttyUSB) - 1] = 0;
  cout << std::string("/dev/") + std::string(ttyUSB) << endl;
  driver = std::make_shared<Driver>(std::string("/dev/") + std::string(ttyUSB) , BaudRate::BAUD_115200);
  if (driver == nullptr)
  {
    std::cout << "Create Uart-Driver Error!" << std::endl;
    return -1;
  }
  // 串口初始化，打开串口设备及配置串口数据格式
  int ret = driver->open();
  if (ret != 0)
  {
    std::cout << "Uart Open failed!" << std::endl;
    return -1;
  }

  ipm.init(Size(COLSIMAGE, ROWSIMAGE),
           Size(COLSIMAGEIPM, ROWSIMAGEIPM)); // IPM逆透视变换初始化
  signal(SIGINT, callbackSignal);             // 程序退出信号

  // PPNC初始化
  PPNCDetection detection;
  if (!detection.init("../res/model/yolov3_mobilenet_v1")) //AI推理初始化
    return 1;


  // 功能类初始化
  ImagePreprocess imagePreprocess;           // 图像预处理类
  TrackRecognition trackRecognition;         // 赛道识别
  ControlCenterCal controlCenterCal;         // 控制中心计算
  MotionController motionController;         // 运动控制
  RingRecognition ringRecognition;           // 环岛识别
  CrossroadRecognition crossroadRecognition; // 十字道路处理
  GarageRecognition garageRecognition;       // 车库识别
  FreezoneRecognition freezoneRecognition;   // 泛型区识别类
  FarmlandDetection farmlandDetection;       // 农田区域检测
  DepotDetection depotDetection;             // 维修厂检测
  GranaryDetection granaryDetection;         // 粮仓检测
  BridgeDetection bridgeDetection;           // 桥梁检测
  SlowZoneDetection slowZoneDetection;       // 慢行区检测
  uint16_t counterRunBegin = 1;              // 智能车启动计数器：等待摄像头图像帧稳定
  uint16_t counterOutTrackA = 0;             // 车辆冲出赛道计数器A
  uint16_t counterOutTrackB = 0;             // 车辆冲出赛道计数器B
  uint16_t circlesThis = 1;                  // 智能车当前运行的圈数
  uint16_t countercircles = 0;               // 圈数计数器
  RoadType roadType = RoadType::BaseHandle;  // 初始化赛道类型
  IcarShow icarShow;                         // 图像显示窗口重绘

  imagePreprocess.imageCorrecteInit(); // 图像矫正参数初始化
  motionController.loadParams();       // 读取配置文件
  trackRecognition.rowCutUp = motionController.params.rowCutUp;
  trackRecognition.rowCutBottom = motionController.params.rowCutBottom;
  garageRecognition.disGarageEntry = motionController.params.disGarageEntry;
  garageRecognition.timeGarageEntry = motionController.params.frameGarageEntry;
  depotDetection.depotStop = motionController.params.depotStop;
  if (motionController.params.GarageEnable) // 出入库使能
    roadType = RoadType::GarageHandle;      // 初始赛道元素为出库

  // 摄像头初始化
  std::string indexCapture = "/dev/video0";
  // if (motionController.params.debug)
  //   indexCapture = "../res/samples/sample.mp4";
  VideoCapture capture(indexCapture);
  if (!capture.isOpened())
  {
    std::cout << "can not open video device " << std::endl;
    return 1;
  }
  capture.set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);

  if (motionController.params.debug) // 调试模式
  {
    //displayWindowInit(); // 显示窗口初始化 //显示窗口初始化
    cv::namedWindow("icar", WINDOW_NORMAL); // 图像名称
    icarShow.init(4);  
  }
  else
  {
    cout << "等待发车!!!" << endl;

    while (!driver->receiveStartSignal())
    {
      ;
    } // 串口接收下位机-比赛开始信号
    cout << "--------- System start!!! -------" << endl;

    driver->sendCMD_enableMotor();

    for (int i = 0; i < 10; i++) // 1秒后发车
    {
      driver->carControl(0, PWMSERVOMID); // 智能车停止运动|建立下位机通信
      waitKey(100);
    }
  }
  int frame_cnt = 1;
  while (1)
  {
    bool imshowRec = false; // 特殊赛道图像显示标志

    Mat frame;
    if (!capture.read(frame))
    {
      std::cout << "no video frame" << std::endl;
      continue;
    }
    // char buffer[64];
    // memset(buffer,0,sizeof(buffer));
    // sprintf(buffer,"%d",frame_cnt++);
    
    // frame = imread("../res/samples/train/" + string(buffer) + ".jpg");
    // cout << "Reading: " << "../res/samples/train/" << string(buffer) << ".jpg" << endl;
    //[00] AI推理
    
    auto feeds = detection.preprocess(frame, {320, 320});

    detection.run(*feeds);
    // get result
    detection.render();

    //[01] 视频源选择
    if (motionController.params.saveImage)
    {
      Mat imageAi = frame.clone();
      detection.drawBox(imageAi);
      //icarShow.setNewWindow(0, imageAi, "imgFrame"); // 添加需要显示的图像
      // savePicture(imageAi);
    }
    else
    {
      // 处理帧时长监测
      static auto preTime = chrono::duration_cast<chrono::milliseconds>(
                                chrono::system_clock::now().time_since_epoch())
                                .count();
      auto startTime = chrono::duration_cast<chrono::milliseconds>(
                           chrono::system_clock::now().time_since_epoch())
                           .count();
      // cout << "run frame time : " << startTime - preTime << "ms" << endl;
      // printf("run frame time: %lld ms\n", startTime - preTime);
      float detFPS = (float)1000.f / (startTime - preTime);
      preTime = startTime;
      if (motionController.params.saveImage) // 保存原始图像
        //savePicture(frame);
        ;
    }

    //[02] 图像预处理
    Mat imgaeCorrect =  frame; //imagePreprocess.imageCorrection(frame);         // RGB
    Mat imageBinary = imagePreprocess.imageBinaryzation(imgaeCorrect); // Gray

    //[03] 基础赛道识别
    trackRecognition.trackRecognition(imageBinary); // 赛道线识别
    if (motionController.params.debug)
    {
      Mat imageTrack = imgaeCorrect.clone();  // RGB
      trackRecognition.drawImage(imageTrack); // 图像显示赛道线识别结果
      icarShow.setNewWindow(1, imageTrack, "imgTrack"); // 添加需要显示的图像
      // savePicture(imageTrack);
    }

    // [04] 出库和入库识别与路径规划
    if (motionController.params.GarageEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::GarageHandle ||
          roadType == RoadType::BaseHandle)
      {
        countercircles++; // 圈数计数
        if (countercircles > 200)
          countercircles = 200;
        if (garageRecognition.startingCheck(detection.results)) // 检测到起点
        {
          bridgeDetection.reset();
          depotDetection.reset();
          farmlandDetection.reset();
          granaryDetection.reset();
          slowZoneDetection.reset();
          crossroadRecognition.reset();
          freezoneRecognition.reset(); // 泛行区识别复位
          ringRecognition.reset();     // 环岛识别初始化

          if (countercircles > 60)
          {
            circlesThis++;
            countercircles = 0;
          }
        }
        printf("circle:%d\n", circlesThis);
         if (circlesThis >= motionController.params.circles &&
             countercircles > 100) // 入库使能：跑完N圈
          garageRecognition.entryEnable = true;

        if (garageRecognition.garageRecognition(trackRecognition,
                                                detection.results))
        {
          if (roadType == RoadType::BaseHandle); // 初次识别-蜂鸣器提醒
            // driver->buzzerSound(1);             // OK

          roadType = RoadType::GarageHandle;
          if (garageRecognition.garageStep ==
              garageRecognition.GarageEntryFinish) // 入库完成
          {
            cout << ">>>>>>>   入库结束 !!!!!" << endl;
            callbackSignal(0);
          }
          if (motionController.params.debug)
          {
            Mat imageGarage =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            garageRecognition.drawImage(trackRecognition, imageGarage);
            icarShow.setNewWindow(2, imageGarage, "imgGarage"); // 添加需要显示的图像
            imshowRec = true;
            // savePicture(imageGarage);
          }
        }
        else
          roadType = RoadType::BaseHandle;

        if (garageRecognition.slowDown) // 入库减速
          slowDownEnable();
      }
    }

    //[05] 农田区域检测
    if (motionController.params.FarmlandEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::FarmlandHandle ||
          roadType == RoadType::BaseHandle)
      {
        if (farmlandDetection.farmlandDetection(trackRecognition,
                                                detection.results))
        {
          if (roadType == RoadType::BaseHandle) ;// 初次识别-蜂鸣器提醒
            // driver->buzzerSound(1);             // OK

          roadType = RoadType::FarmlandHandle;
          if (motionController.params.debug)
          {
            Mat imageFarmland =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            farmlandDetection.drawImage(trackRecognition, imageFarmland);
            icarShow.setNewWindow(2, imageFarmland, "imgFarmland"); // 添加需要显示的图像
            imshowRec = true;
            //savePicture(imageFarmland);
          }
          cout << "enter farmland!" << endl;
        }
        else
          roadType = RoadType::BaseHandle;
      }
    }

    //[06] 维修厂检测
    if (motionController.params.DepotEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::DepotHandle ||
          roadType == RoadType::BaseHandle)
      {
        if (depotDetection.depotDetection(trackRecognition,
                                          detection.results))
        {
          if (roadType == RoadType::BaseHandle); // 初次识别-蜂鸣器提醒
            // driver->buzzerSound(1);             // OK
          if (depotDetection.depotStep == depotDetection.DepotStep::DepotStop)
          {
            static int counter = 1;
            string img_path = "../res/samples/depotStop";
            string name = img_path + to_string(counter) + ".jpg";
            imwrite(name, frame);
            counter ++;
          }
          roadType = RoadType::DepotHandle;
          if (motionController.params.debug)
          {
            
            ;
          }
          cout << "enter depot!" << endl;
        }
        else
          roadType = RoadType::BaseHandle;
      }
    }

    //[07] 粮仓检测
    if (motionController.params.GranaryEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::GranaryHandle ||
          roadType == RoadType::BaseHandle)
      {
        if (granaryDetection.granaryDetection(trackRecognition,
                                              detection.results))
        {
          if (roadType == RoadType::BaseHandle); // 初次识别-蜂鸣器提醒
            // driver->buzzerSound(1);             // OK

          roadType = RoadType::GranaryHandle;
          if (motionController.params.debug)
          {
            Mat imageGranary =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            granaryDetection.drawImage(trackRecognition, imageGranary);
            icarShow.setNewWindow(2, imageGranary, "imgGranary"); // 添加需要显示的图像
            imshowRec = true;
            savePicture(imageGranary);
          }
        }
        else
          roadType = RoadType::BaseHandle;
      }
    }

    // [08] 坡道（桥）检测与路径规划
    if (motionController.params.BridgeEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::BridgeHandle ||
          roadType == RoadType::BaseHandle)
      {
        if (bridgeDetection.bridgeDetection(trackRecognition,
                                            detection.results))
        {
          if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
            // driver->buzzerSound(1);             // OK

          roadType = RoadType::BridgeHandle;
          if (motionController.params.debug)
          {
            Mat imageBridge =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            bridgeDetection.drawImage(trackRecognition, imageBridge);
            icarShow.setNewWindow(2, imageBridge, "imgBridge"); // 添加需要显示的图像
            imshowRec = true;
            savePicture(imageBridge);
          }
          cout << "enter bridge!" << endl;
        }
        else
          roadType = RoadType::BaseHandle;
      }
    }

    // [09] 慢行区检测与路径规划
    if (motionController.params.SlowzoneEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::SlowzoneHandle ||
          roadType == RoadType::BaseHandle)
      {
        if (slowZoneDetection.slowZoneDetection(trackRecognition,
                                                detection.results))
        {
          if (roadType == RoadType::BaseHandle); // 初次识别-蜂鸣器提醒
            // driver->buzzerSound(1);             // OK

          roadType = RoadType::SlowzoneHandle;
          if (motionController.params.debug)
          {
            Mat imageSlow =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            slowZoneDetection.drawImage(trackRecognition, imageSlow);
            icarShow.setNewWindow(2, imageSlow, "imgSlow"); // 添加需要显示的图像
            imshowRec = true;
            savePicture(imageSlow);
          }
          cout << "enter slowzone!" << endl;
        }
        else
          roadType = RoadType::BaseHandle;
      }
    }

    // [10] 泛行区检测与识别
    if (motionController.params.FreezoneEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::FreezoneHandle ||
          roadType == RoadType::BaseHandle)
      {
        if (freezoneRecognition.freezoneRecognition(trackRecognition))
        {
          if (roadType == RoadType::BaseHandle); // 初次识别-蜂鸣器提醒
            // driver->buzzerSound(1);             // OK

          roadType = RoadType::FreezoneHandle;
          if (motionController.params.debug)
          {
            Mat imageFreezone =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            freezoneRecognition.drawImage(trackRecognition, imageFreezone);
            icarShow.setNewWindow(2, imageFreezone, "imgFreezone"); // 添加需要显示的图像
            imshowRec = true;
            savePicture(imageFreezone);
          }
        }
        else
          roadType = RoadType::BaseHandle;
      }
    }

    // [11] 环岛识别与处理
    if (motionController.params.RingEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::RingHandle ||
          roadType == RoadType::BaseHandle)
      {
        if (ringRecognition.ringRecognition(trackRecognition, imageBinary))
        {
          if (roadType == RoadType::BaseHandle) ;// 初次识别-蜂鸣器提醒
            // driver->buzzerSound(1);             // OK

          roadType = RoadType::RingHandle;
          if (motionController.params.debug)
          {
            Mat imageRing =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            ringRecognition.drawImage(trackRecognition, imageRing);
            icarShow.setNewWindow(2, imageRing, "imgRing"); // 添加需要显示的图像
            imshowRec = true;
            savePicture(imageRing);
            cout << "enter ring!" << endl;
          }
        }
        else
          roadType = RoadType::BaseHandle;
      }
    }

    // [12] 十字道路处理
    if (motionController.params.CrossEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::CrossHandle ||
          roadType == RoadType::BaseHandle)
      {
        if (crossroadRecognition.crossroadRecognition(trackRecognition,
                                                      detection.results))
        {
          roadType = RoadType::CrossHandle;

          if (motionController.params.debug)
          {
            Mat imageCross =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            crossroadRecognition.drawImage(trackRecognition, imageCross);
            icarShow.setNewWindow(2, imageCross, "imgCross"); // 添加需要显示的图像
            imshowRec = true;
            savePicture(imageCross);
          }
          cout << "enter cross!" << endl;
        }
        else
          roadType = RoadType::BaseHandle;
      }
    }

    // [13] 控制中心计算
    if (trackRecognition.pointsEdgeLeft.size() < 30 &&
        trackRecognition.pointsEdgeRight.size() < 30 &&
        roadType != RoadType::BridgeHandle &&
        roadType != RoadType::GranaryHandle &&
        roadType != RoadType::DepotHandle &&
        roadType != RoadType::FarmlandHandle) // 防止车辆冲出赛道
    {
      counterOutTrackA++;
      counterOutTrackB = 0;
      if (counterOutTrackA > 20)
        callbackSignal(0);
    }
    else
    {
      counterOutTrackB++;
      if (counterOutTrackB > 50)
      {
        counterOutTrackA = 0;
        counterOutTrackB = 50;
      }
    }
    controlCenterCal.controlCenterCal(
        trackRecognition); // 根据赛道边缘信息拟合运动控制中心

    // [14] 运动控制
    if (counterRunBegin > 30) ////智能车启动延时：前几场图像不稳定
    {
      // 智能汽车方向控制
      motionController.pdController(
          controlCenterCal.controlCenter); // PD控制器姿态控制

      // 智能汽车速度控制
      switch (roadType)
      {
      case RoadType::GarageHandle:
        motionController.motorSpeed =
            motionController.params.speedGarage; // 匀速控制
        break;
      case RoadType::BridgeHandle:
        motionController.motorSpeed =
            motionController.params.speedBridge; // 匀速控制
        break;
      case RoadType::SlowzoneHandle:
        motionController.motorSpeed =
            motionController.params.speedSlowzone; // 匀速控制
        break;
      case RoadType::FarmlandHandle:
        motionController.motorSpeed =
            motionController.params.speedSlowzone; // 匀速控制
        break;
      default: // 基础巡线 | 十字 |环岛速度
        motionController.speedController(true, slowDown,
                                         controlCenterCal); // 变加速控制
        break;
      }

      if (!motionController.params.debug) // 调试模式下不控制车辆运动
      {
        if(DEBUG_RING_EXIT){
          driver->carControl(0, 0);
        }
        else{
            if (roadType == RoadType::DepotHandle)
        {
          if (depotDetection.depotStep == 4)
            driver->carControl(
                0, motionController.servoPwm); // 串口通信，姿态与速度控制
          else if (depotDetection.depotStep == 5)
            driver->carControl(
                -0.3,
                motionController.servoPwm); // 串口通信，姿态与速度控制
          else
            driver->carControl(
                0.3,
                motionController.servoPwm); // 串口通信，姿态与速度控制
        }
        else
        {
          driver->carControl(
              motionController.motorSpeed,
              motionController.servoPwm);
              } // 串口通信，姿态与速度控制
        }
      }

      // 减速缓冲
      if (slowDown)
      {
        counterSlowDown++;
        if (counterSlowDown > 50)
        {
          slowDown = false;
          counterSlowDown = 0;
        }
      }
    }
    else
      counterRunBegin++;

    // [15]调试模式下图像显示和存图
    if (true)
    {
      controlCenterCal.drawImage(trackRecognition, imgaeCorrect);
      switch (roadType)
      {
      case RoadType::BaseHandle: // 基础赛道处理
        putText(imgaeCorrect, "[1] Track", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1,
                CV_AA); // 显示赛道识别类型
        break;
      case RoadType::RingHandle: // 环岛赛道处理
        putText(imgaeCorrect, "[2] Ring", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA); // 显示赛道识别类型
        break;
      case RoadType::CrossHandle: // 十字道路处理
        putText(imgaeCorrect, "[3] Cross", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA); // 显示赛道识别类型
        break;
      case RoadType::FreezoneHandle: // 泛行区处理
        putText(imgaeCorrect, "[4] Freezone", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA); // 显示赛道识别类型
        break;
      case RoadType::GarageHandle: // 车库处理
        putText(imgaeCorrect, "[5] Garage", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA); // 显示赛道识别类型
        break;
      case RoadType::GranaryHandle: // 粮仓处理
        putText(imgaeCorrect, "[6] Granary", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA); // 显示赛道识别类型
        break;
      case RoadType::DepotHandle: // 修车厂处理
        putText(imgaeCorrect, "[7] Depot", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA); // 显示赛道识别类型
        break;
      case RoadType::BridgeHandle: // 坡道(桥)处理
        putText(imgaeCorrect, "[8] Bridge", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA); // 显示赛道识别类型
        break;
      case RoadType::SlowzoneHandle: // 慢行区（动物出没）处理
        putText(imgaeCorrect, "[9] Slowzone", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA); // 显示赛道识别类型
        break;
      case RoadType::FarmlandHandle: // 农田区域处理
        putText(imgaeCorrect, "[10] Farmland", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA); // 显示赛道识别类型
        break;
      }

      putText(imgaeCorrect, "Error: " + to_string(controlCenterCal.controlCenter - COLSIMAGE / 2), Point(10, 40),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1,
                CV_AA); // 显示赛道识别类型

      putText(imgaeCorrect,
              "v: " + formatDoble2String(motionController.motorSpeed, 2),
              Point(COLSIMAGE - 60, 80), FONT_HERSHEY_PLAIN, 1,
              Scalar(0, 0, 255), 1); // 车速

      string str = to_string(circlesThis) + "/" +
                   to_string(motionController.params.circles);
      putText(imgaeCorrect, str, Point(COLSIMAGE - 50, ROWSIMAGE - 20),
              cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
              CV_AA); // 显示圈数
      if (!imshowRec) // 保持调试图像存储顺序和显示一致性
      {
        Mat imageNone = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
        icarShow.setNewWindow(2, imageNone, "None"); // 添加需要显示的图像
        // savePicture(imageNone);

      }
      icarShow.setNewWindow(3, imgaeCorrect, "imgCorrect"); // 添加需要显示的图像
      if(motionController.params.saveImage)
        savePicture(imgaeCorrect);
      /* debug ring exit save picture */
      if (DEBUG_RING_EXIT) {
        DEBUG_RING_EXIT = false;
        imwrite("../res/samples/ringExit.jpg", imgaeCorrect);
        //waitKey(0);
      }
      icarShow.display(); // 图像窗口显示
      char c = waitKey(1);
    }
  }
  return 0;
}

/**
 * @brief 系统信号回调函数：系统退出
 *
 * @param signum 信号量
 */
void callbackSignal(int signum)
{
  driver->carControl(0, PWMSERVOMID); // 智能车停止运动
  cout << "====System Exit!!!  -->  CarStopping! " << signum << endl;
  exit(signum);
}

/**
 * @brief OpenCV图像显示窗口初始化（详细参数/Debug模式）
 *
 */
void displayWindowInit(void)
{
  //   //[1] 二值化图像：Gray
  //   string windowName = "imageTrack";
  //   cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
  //   cv::resizeWindow(windowName, 320, 240);     // 分辨率
  //   cv::moveWindow(windowName, 10, 10);         // 布局位置

  //   //[2] 赛道边缘图像：RGB
  //   windowName = "imageRecognition";
  //   cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
  //   cv::resizeWindow(windowName, 320, 240);     // 分辨率
  //   cv::moveWindow(windowName, 10, 320);        // 布局位置

  //   //[3] 原始图像/矫正后：RGB
  //   windowName = "imageControl";
  //   cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
  //   cv::resizeWindow(windowName, 640, 480);     // 分辨率
  //   cv::moveWindow(windowName, 350, 20);        // 布局位置
}

/**
 * @brief 车辆减速使能
 *
 */
void slowDownEnable(void)
{
  slowDown = true;
  counterSlowDown = 0;
}