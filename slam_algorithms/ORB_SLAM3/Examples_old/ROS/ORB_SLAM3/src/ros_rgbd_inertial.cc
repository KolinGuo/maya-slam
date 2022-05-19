/**
* This file is part of ORB-SLAM3
*
* CopyDepth (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* CopyDepth (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
    

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bRect, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), do_rectify(bRect), mbClahe(bClahe){}

    void GrabImageRGB(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageDepth(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg, int flag);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgRGBBuf, imgDepthBuf;
    std::mutex mBufMutexRGB,mBufMutexDepth;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "RGBD_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if(argc < 4 || argc > 5)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD_Inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }

  std::string sbRect(argv[3]);
  if(argc==5)
  {
    std::string sbEqual(argv[4]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_RGBD,true);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,sbRect == "true",bEqual);
  
    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["RGB.K"] >> K_l;
        fsSettings["Depth.K"] >> K_r;

        fsSettings["RGB.P"] >> P_l;
        fsSettings["Depth.P"] >> P_r;

        fsSettings["RGB.R"] >> R_l;
        fsSettings["Depth.R"] >> R_r;

        fsSettings["RGB.D"] >> D_l;
        fsSettings["Depth.D"] >> D_r;

        int rows_l = fsSettings["RGB.height"];
        int cols_l = fsSettings["RGB.width"];
        int rows_r = fsSettings["Depth.height"];
        int cols_r = fsSettings["Depth.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

  // Maximum delay, 5 seconds
  cout << "\nSetting up Subscribers \n";
  ros::Subscriber sub_imu = n.subscribe("/camera/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
  ros::Subscriber sub_img_rgb = n.subscribe("/camera/color/image_raw", 100, &ImageGrabber::GrabImageRGB,&igb);
  ros::Subscriber sub_img_depth = n.subscribe("/camera/aligned_depth_to_color/image_raw", 100, &ImageGrabber::GrabImageDepth,&igb);

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);
  ros::spin();
  return 0;


}



void ImageGrabber::GrabImageRGB(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexRGB.lock();
  if (!imgRGBBuf.empty())
    imgRGBBuf.pop();
  imgRGBBuf.push(img_msg);
  mBufMutexRGB.unlock();
}

void ImageGrabber::GrabImageDepth(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexDepth.lock();
  if (!imgDepthBuf.empty())
    imgDepthBuf.pop();
  imgDepthBuf.push(img_msg);
  cout<<"Depth image received! " << std::endl;
  mBufMutexDepth.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg, int flag = 0)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    if(flag == 0)
      cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    else
      cv_ptr = cv_bridge::toCvShare(img_msg);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  return cv_ptr->image;
  
  // if(cv_ptr->image.type()==0)
  // {
  //   return cv_ptr->image.clone();
  // }
  // else
  // {
  //   std::cout << "Error type" << std::endl;
  //   return cv_ptr->image.clone();
  // }
}


void ImageGrabber::SyncWithImu()
{
  const double maxTimeDiff = 0.01;
  while(1)
  {
    cv::Mat imRGB, imDepth;
    double tImRGB = 0, tImDepth = 0;
    if (!imgRGBBuf.empty()&&!imgDepthBuf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tImRGB = imgRGBBuf.front()->header.stamp.toSec();
      tImDepth = imgDepthBuf.front()->header.stamp.toSec();

      this->mBufMutexDepth.lock();
      while((tImRGB-tImDepth)>maxTimeDiff && imgDepthBuf.size()>1)
      {
        imgDepthBuf.pop();
        tImDepth = imgDepthBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexDepth.unlock();

      this->mBufMutexRGB.lock();
      while((tImDepth-tImRGB)>maxTimeDiff && imgRGBBuf.size()>1)
      {
        imgRGBBuf.pop();
        tImRGB = imgRGBBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexRGB.unlock();

      if((tImRGB-tImDepth)>maxTimeDiff || (tImDepth-tImRGB)>maxTimeDiff)
      {
        std::cout << "big time difference" << std::endl;
        continue;
      }
      if(tImRGB>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;

      this->mBufMutexRGB.lock();
      imRGB = GetImage(imgRGBBuf.front());
      imgRGBBuf.pop();
      this->mBufMutexRGB.unlock();

      this->mBufMutexDepth.lock();
      imDepth = GetImage(imgDepthBuf.front(), 1);
      imgDepthBuf.pop();
      this->mBufMutexDepth.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImRGB)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
      {
        mClahe->apply(imRGB,imRGB);
        mClahe->apply(imDepth,imDepth);
      }

      if(do_rectify)
      {
        cv::remap(imRGB,imRGB,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imDepth,imDepth,M1r,M2r,cv::INTER_LINEAR);
      }

      mpSLAM->TrackRGBD(imRGB,imDepth,tImRGB,vImuMeas);

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}


