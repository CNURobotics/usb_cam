/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Robert Bosch nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>

namespace usb_cam {

class UsbCamNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_;
  image_transport::CameraPublisher image_pub_;

  ros::Subscriber sync_sub_;
  ros::Publisher  sync_pub_;
  ros::Publisher  sync_data_pub_;
  geometry_msgs::PoseStamped sync_data_;


  // parameters
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_, camera_sync_topic_;
  double camera_sync_threshold_, camera_sync_max_adjustment_,average_sync_diff_;
  ros::Time  desired_target_;
  ros::Duration adjust_sync_;
  bool camera_publish_sync_ = false;


  //std::string start_service_name_, start_service_name_;
  bool streaming_status_;
  int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
      white_balance_, gain_;
  bool autofocus_, autoexposure_, auto_white_balance_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
  bool grab_image_;

  UsbCam cam_;

  ros::ServiceServer service_start_, service_stop_;



  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.start_capturing();
    return true;
  }


  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.stop_capturing();
    return true;
  }

  UsbCamNode() :
      node_("~")
  {
    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);

    // grab the parameters
    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    node_.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
    node_.param("contrast", contrast_, -1); //0-255, -1 "leave alone"
    node_.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
    node_.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
    // possible values: mmap, read, userptr
    node_.param("io_method", io_method_name_, std::string("mmap"));
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 30);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
    // enable/disable autofocus
    node_.param("autofocus", autofocus_, false);
    node_.param("focus", focus_, -1); //0-255, -1 "leave alone"
    // enable/disable autoexposure
    node_.param("autoexposure", autoexposure_, true);
    node_.param("exposure", exposure_, 100);
    node_.param("gain", gain_, -1); //0-100?, -1 "leave alone"
    // enable/disable auto white balance temperature
    node_.param("auto_white_balance", auto_white_balance_, true);
    node_.param("white_balance", white_balance_, 4000);

    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    node_.param("camera_name", camera_name_, std::string("head_camera"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));
    node_.param("camera_sync_topic", camera_sync_topic_, std::string(""));
    node_.param("camera_sync_threshold", camera_sync_threshold_, 0.01/framerate_); // 1% of desired framerate
    node_.param("camera_sync_max_adjustment", camera_sync_max_adjustment_, 0.5/framerate_); // 50% of desired framerate
    node_.param("camera_publish_sync", camera_publish_sync_, false);

    average_sync_diff_ = 0.0;

    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

    // create Services
    service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
    service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);

    // check for default camera info
    if (!cinfo_->isCalibrated())
    {
      cinfo_->setCameraName(video_device_name_);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = img_.header.frame_id;
      camera_info.width = image_width_;
      camera_info.height = image_height_;
      cinfo_->setCameraInfo(camera_info);
    }


    ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
        image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

    // set the IO method
    UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
    if(io_method == UsbCam::IO_METHOD_UNKNOWN)
    {
      ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
      node_.shutdown();
      return;
    }

    // set the pixel format
    UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
    if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
    {
      ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
      node_.shutdown();
      return;
    }

    // start the camera
    cam_.start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
		     image_height_, framerate_);

    // set camera parameters
    if (brightness_ >= 0)
    {
      cam_.set_v4l_parameter("brightness", brightness_);
    }

    if (contrast_ >= 0)
    {
      cam_.set_v4l_parameter("contrast", contrast_);
    }

    if (saturation_ >= 0)
    {
      cam_.set_v4l_parameter("saturation", saturation_);
    }

    if (sharpness_ >= 0)
    {
      cam_.set_v4l_parameter("sharpness", sharpness_);
    }

    if (gain_ >= 0)
    {
      cam_.set_v4l_parameter("gain", gain_);
    }

    // check auto white balance
    if (auto_white_balance_)
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
      cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
    }

    // check auto exposure
    if (!autoexposure_)
    {
      // turn down exposure control (from max of 3)
      cam_.set_v4l_parameter("exposure_auto", 1);
      // change the exposure level
      cam_.set_v4l_parameter("exposure_absolute", exposure_);
    }

    // check auto focus
    if (autofocus_)
    {
      cam_.set_auto_focus(1);
      cam_.set_v4l_parameter("focus_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("focus_auto", 0);
      if (focus_ >= 0)
      {
        cam_.set_v4l_parameter("focus_absolute", focus_);
      }
    }

    grab_image_ = true;

    if (camera_sync_topic_ != "")
    {
        if (camera_publish_sync_)
        {
           sync_pub_ = node_.advertise<std_msgs::Header>(camera_sync_topic_,5);
           ROS_INFO(" Advertise camera sync topic <%s>",camera_sync_topic_.c_str());
        }
        else
        {
           sync_sub_ = node_.subscribe(camera_sync_topic_,10,&UsbCamNode::sync_callback,this);
           ROS_INFO(" Subscribe to camera sync topic <%s>",camera_sync_topic_.c_str());
           sync_data_pub_ = node_.advertise<geometry_msgs::PoseStamped>(camera_sync_topic_+"_data",5);
           ROS_INFO(" Advertise camera sync data topic <%s_data>",camera_sync_topic_.c_str());
        }
    }
    else
    {
        camera_publish_sync_ = false;
    }
  }

  virtual ~UsbCamNode()
  {
    cam_.shutdown();
  }

  bool take_and_send_image()
  {
    // grab the image
    cam_.grab_image(&img_);

    // grab the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci->header.frame_id = img_.header.frame_id;
    ci->header.stamp = img_.header.stamp;

    // publish the image
    image_pub_.publish(img_, *ci);
    if (camera_publish_sync_)
    {
        sync_pub_.publish(img_.header);
    }

    return true;
  }

  bool spin()
  {
    if (camera_sync_topic_== "" || camera_publish_sync_ == true)
    { // Standard operation
        ros::Rate loop_rate(this->framerate_);
        while (node_.ok())
        {
          if (cam_.is_capturing()) {
            if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");
          }
          ros::spinOnce();
          loop_rate.sleep();

        }
        return true;
    }
    else
    { // Calculate the cycle time in order to remain approximately synchronized with another camera

        ros::Rate loop_rate(2000);

        while (node_.ok())
        {
          if (cam_.is_capturing()) {
            if (ros::Time::now() >= desired_target_)
            {
                grab_image_ = false; // already grabbed the image corresponding to the prior target
                if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");

                desired_target_ = cam_.last_sent() + ros::Duration(2.0/framerate_);      // set target for next grab at half desired rate (twice time period) - expect sync to correct first
                sync_data_.pose.position.x += 1.0;
                sync_data_.pose.position.y += loop_rate.cycleTime().toSec();
                sync_data_.pose.position.z += loop_rate.expectedCycleTime().toSec();
                sync_data_.header.stamp = ros::Time::now();
                sync_data_pub_.publish(geometry_msgs::PoseStamped(sync_data_));
                sync_data_ = geometry_msgs::PoseStamped();
            }
            else
            {
                sync_data_.pose.position.x += 1.0;
                sync_data_.pose.position.y += loop_rate.cycleTime().toSec();
                sync_data_.pose.position.z += loop_rate.expectedCycleTime().toSec();
            }
          }
          ros::spinOnce(); // this is where the sync call back is processed
          loop_rate.sleep();

        }
        return true;

    }
  }


  inline void sync_callback(const std_msgs::Header::ConstPtr& msg)
  {
      // This is called upon receipt of sync message from the master camera
      // There are two cases we will consider
      // either 1) we got a new sync message after we send out our latest,
      //  or    2) we get the sync while waiting up send our corresponding image


      //ROS_INFO(" received sync topic = %f",msg->stamp.toSec());
      double diff = (msg->stamp - cam_.last_sent()).toSec();
      double raw_diff = diff;
      if (fabs(diff) < camera_sync_threshold_)
      { // must have just grabbed, so we are doing well


          average_sync_diff_ = 0.9*average_sync_diff_ + 0.1*diff;
          adjust_sync_       = ros::Duration(average_sync_diff_);
          desired_target_ = msg->stamp + ros::Duration(1.0/framerate_) + adjust_sync_; // reset the target so that we match the master camera
          grab_image_  = true;  // Need to grab the corresponding image
      }
      else if (diff > 0.0)
      {
          // It is possible that we grabbed slightly ahead of the master based on sync, in which case diff is positive, but we don't need to grab
          // Use the grab_image_ flag to disambiguate the conditions

          // we are expected to grab soon
          if (cam_.is_capturing() && grab_image_) {
                if (!take_and_send_image())
                {
                            ROS_WARN("USB camera did not respond in time.");
                }
                diff = (msg->stamp - cam_.last_sent()).toSec(); // this will be negative, which implies grab earlier next time to compensate for lag
                average_sync_diff_ = 0.9*average_sync_diff_ + 0.1*diff;
                adjust_sync_       = ros::Duration(average_sync_diff_);
                desired_target_ = msg->stamp + ros::Duration(1.0/framerate_) + adjust_sync_; // specify the target for next grab
                grab_image_     = true; // flag to grab the next image

                sync_data_.header.stamp = ros::Time::now();
                sync_data_pub_.publish(geometry_msgs::PoseStamped(sync_data_));
                sync_data_ = geometry_msgs::PoseStamped();
          }
      }
      else
      {   // means camera master grabbed earlier, assuming we don't have a significant comms transport delay  then  need to debug

          if (diff < -camera_sync_max_adjustment_)
          {
              ROS_WARN("Camera  sync diff=%f < threshold (avg=%f) - desired_target = %f = significant offset - reset tuning sync=%f offset=%f",diff, average_sync_diff_, desired_target_.toSec(),msg->stamp.toSec(),adjust_sync_.toSec());
              desired_target_ = msg->stamp + ros::Duration(1.0/framerate_);
              grab_image_     = true;
              adjust_sync_     = ros::Duration();
              average_sync_diff_ = 0.0; // reset
          }
          else
          {
              // Somewhere between really late comms and good sync; presume our sync is working
              average_sync_diff_ = 0.9*average_sync_diff_ + 0.1*diff;
              adjust_sync_       = ros::Duration(1.1*average_sync_diff_); // slightly over correct
              desired_target_ = msg->stamp + ros::Duration(1.0/framerate_)  + adjust_sync_;
              grab_image_     = true;
          }
      }

      sync_data_.pose.orientation.x = average_sync_diff_;
      sync_data_.pose.orientation.y = adjust_sync_.toSec();
      sync_data_.pose.orientation.z = diff;
      sync_data_.pose.orientation.w = raw_diff;

  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  usb_cam::UsbCamNode a;
  a.spin();
  return EXIT_SUCCESS;
}
