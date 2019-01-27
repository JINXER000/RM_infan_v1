//
// Author: Yizhou chen
//


#include "MarkerSensor.h"
#include "kalman.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <stdio.h>
#include <thread>

//#define _DBG1FRAME_

#ifndef _WIN32
#include "MVCamera.h"
#include "serial.h"
#include "Timer.h"
#include "videosaver.h"

#endif // TX2
using namespace std;
using namespace cv;


Size dist_size = Size(640, 480);
int X_bias, Y_bias, pix_x, pix_y;
int status=0,lastStatus=0;
bool issave = 0;
int camWay = 2; // 0: MVcamera, 1: usb cam  2: vedio
#ifndef _WIN32
// vector<int> CalculateXYPixel(Rect location);
int messnum = 0;
bool transmit_message = 0;
Serial sel;
unsigned char data_send_buf[8];
////--------------------------------------
unsigned char Add_CRC(unsigned char InputBytes[], unsigned char data_lenth) {
  unsigned char byte_crc = 0;
  for (unsigned char i = 0; i < data_lenth; i++) {
    byte_crc += InputBytes[i];
  }
  return byte_crc;
}

void Data_disintegrate(unsigned int Data, unsigned char *LData,
                       unsigned char *HData) {
  *LData = Data & 0XFF;          // 0xFF = 1111 1111
  *HData = (Data & 0xFF00) >> 8; // 0xFF00 = 1111 1111 0000 0000
}
//
void Data_Code(unsigned int x_Data, unsigned int y_Data) {
  int length = 8;
  data_send_buf[0] = 0xFF;
  data_send_buf[1] = length;
  data_send_buf[2] = 0x02;
  //
  Data_disintegrate(x_Data, &data_send_buf[3], &data_send_buf[4]);
  Data_disintegrate(y_Data, &data_send_buf[5], &data_send_buf[6]);
  //
  data_send_buf[length - 1] = Add_CRC(data_send_buf, length - 1);
}
////----------------------

#endif // TX2

string num2str(double i)

{
  stringstream ss;
  ss << i;
  return ss.str();
}
void sel_send() {
  while (1) {
#ifndef _WIN32
    // transmit msg
    if (transmit_message == 1) {
      if (status == 1) {

        char buff[8];
        messnum++;
        Data_Code(X_bias, Y_bias);
        cout << "x_cord= " << X_bias << "    y_cord=  " << Y_bias << endl;
        for (int i = 0; i < 8; i++)
          buff[i] = data_send_buf[i];
        sel.writeData(buff, 8);
          cout << "     X_bias:  " << X_bias << "   Y_bias:    " << Y_bias<< endl;
      } else {
        char buff[8];
        messnum++;
        Data_Code(850, 850);
        cout << "status=== " << status << endl;
        for (int i = 0; i < 8; i++)
          buff[i] = data_send_buf[i];
        sel.writeData(buff, 8);
      }
    }

#else
    // cout << "vs_test" << endl;

#endif // TX2
    this_thread::sleep_for(chrono::milliseconds(1));
  }
}
int frame_process() {

  Mat srcImg, bgrImg;
  bool isSuccess = 0;
  float X = 0, Y = 0, Z = 0;
  int led_type = 0, false_idx = 0, frameCnt = 0, capIdx = 1;
  MarkSensor markSensor;
  long timeStamp[2];
  HaarD haarDetector;
  bool ishaar = 0;
#ifndef _WIN32
  VideoSaver saver;

  bool auto_wb;
  double exp_time = 10000;
  bool auto_exp = false;
  bool large_resolution = false;
#endif
  VideoCapture capture;
  if (camWay == 1) {

    capture.open(capIdx);
    if (!capture.isOpened()) {
      cout << "cap get error!\n";

      abort();
    }
  } else {

    capture.open("../video/red-mark17.mov");
  }
  /// Load cascade
  String cascade_name = "../params/zgdcascade_1.xml";
  if (!haarDetector.detector.load(cascade_name)) {
    printf("--(!)Error loading face cascade\n");
  };

  //init kalman filter

  const int stateNum=4;                                      //state var(x,y,△x,△y)
  const int measureNum=2;                                    //measure var(x,y)
  float dt=1/100;
  Eigen::MatrixXd A(stateNum, stateNum); // System dynamics matrix
  Eigen::MatrixXd C(measureNum, stateNum); // Output matrix
  Eigen::MatrixXd Q(stateNum, stateNum); // Process noise covariance
  Eigen::MatrixXd R(measureNum, measureNum); // Measurement noise covariance
  Eigen::MatrixXd P(stateNum, stateNum); // Estimate error covariance
  A<<1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1;
  C<<1,0,0,0,0,1,0,0;
//   C.setIdentity();
  Q.setIdentity();
  Q=Q*1e-5;
  R.setIdentity();
  R=R*1e-1;
  P.setIdentity();
  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;
  Eigen::VectorXd xs0(stateNum); //unset 
  Eigen::VectorXd ym(measureNum); 
  Kalman_Filter kf(dt,A,C,Q,R,P);
  
  // opencv kalman filter

//   cvKalmanFilter cvkf(0,0);
  Point kfPoint2,kf_pt,finalPt;
  
  timeStamp[1] = getTickCount();
  while (true) {
    capture.read(srcImg);

    frameCnt++;
    if (srcImg.empty()) {
      printf("Image empty !\n");
      continue;
    }
    /// calc fps
    timeStamp[0] = getTickCount();
    float timeFly = (timeStamp[0] - timeStamp[1]) / getTickFrequency();
    float fps = 1 / timeFly;
    timeStamp[1] = getTickCount();
    cout << "-----FPS------" << fps << endl;


    /// detect and track
    resize(srcImg, bgrImg, dist_size);
    // bgrImg=srcImg.clone();

    if (!ishaar)
      isSuccess = markSensor.ProcessFrameLEDXYZ(bgrImg, X, Y, Z, led_type,
                                                pix_x, pix_y);
    else
      isSuccess =
          haarDetector.Detect_track(bgrImg, X, Y, Z, led_type, pix_x, pix_y);


	  
  if (!isSuccess) {
      status = 1;
      cout << "detected target--------------" << endl;
      X_bias = pix_x - bgrImg.cols/2;
      Y_bias = pix_y - bgrImg.rows/2;

	if(lastStatus==0)   //new target !
      {
        xs0<<X_bias,Y_bias,0,0;
	kf.init(timeStamp[1],xs0);
	
      }else
      {
	ym<<X_bias,Y_bias;
	kf.update(ym);
      }
      //2 kalman filter output
      kf_pt=Point(kf.state()(0)+bgrImg.cols/2,kf.state()(1)+bgrImg.rows/2);

      finalPt=Point(pix_x,pix_y);
      // kalman filter actually lag, so add a pid to tune... aweful idea actually 
      //refer to https://blog.csdn.net/m0_37857300/article/details/79117062
      Point diff=finalPt-kf_pt;
      if(norm(diff)>5)
      {
	finalPt=finalPt+1*diff;
      }

    } else {
      
      status = 0;
      cout << "detected failed!!!!!!!!!!!!!!!!!!!!!!!" << endl;
      
//       kf.lostAndReset(50);
    }

    
    
    lastStatus=status;

    /// write demo
    // VideoWriter writer;	//?????????????????��VideoWriter??????
    // string videoPathCut =
    // "F://vs_ws//cpp_tutorial//RM_zju//demo_save//demo.avi";
    // writer.open(videoPathCut, CV_FOURCC('D', 'I', 'V', 'X'), rate,
    // videoSize); writer << MarkSensor::img_show;
    /// show img
    if (MarkerParams::ifShow) {
      if (MarkSensor::img_show.empty()) {
        break;
      }

      string recInfo = "";

      if (issave)
        recInfo = "recording";
      string frameCnt_s = recInfo + num2str(frameCnt);
      putText(MarkSensor::img_show, frameCnt_s, cv::Point(500, 20),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
      string fpsString("FPS:");
      string fps_string = num2str(fps);
      fpsString += fps_string;
      putText(MarkSensor::img_show, fpsString, cv::Point(5, 20),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
      


      circle(MarkSensor::img_show, kf_pt, 5, Scalar(0, 0, 255),4); 
      circle(MarkSensor::img_show, finalPt, 5, Scalar(255, 0, 0),4);
      imshow("all markers", MarkSensor::img_show);

      char key = waitKey(1);
      if (key == 27 || key == 'q' || key == 'Q') {
        break;
        // writer.release();
      }
      if (key == 'z')
        system("pause");
      if (key == 's') {
        false_idx++;
        string saveName_src =
            "../dbg_imgs/" + num2str(false_idx) + "falsesrc.jpg";
        imwrite(saveName_src, srcImg);
        string saveName_post =
            "../dbg_imgs/" + num2str(false_idx) + "false_post.jpg";
        imwrite(saveName_post, MarkSensor::img_show);
      }

      if (key == 'h') {
        ishaar = !ishaar;
      }
      if (key == 'r') {
        issave = !issave;
      }
    }
#ifndef _WIN32
    if (issave) {
      saver.write(MarkSensor::img_show);
    }
#endif
  }

  return 0;
}

int main()
{
#ifndef _WIN32

	//set param of sel

	sel.setPara();
#endif 
	thread th1(&frame_process);
	thread th2(&sel_send);
	th2.join();
	th1.join();

}
