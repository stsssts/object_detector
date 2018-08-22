#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <dynamic_reconfigure/server.h>
#include <object_detector/ColorBoundsConfig.h>


class ColorCalibrator
{
public:
  ColorCalibrator(ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh("~");

    int fps;
    bool use_topic;
    std::string cam_id;
    pnh.param("camera_id", cam_id, std::string("0"));
    pnh.param("use_topic", use_topic, false);
    pnh.param("fps", fps, 30);

    server_.setCallback(boost::bind(&ColorCalibrator::reconfigureCallback, this, _1, _2));

    if (!use_topic)
    {
      initializeCamera_(cam_id);
      timer_ = nh.createTimer(ros::Rate(fps), &ColorCalibrator::cvImageCallback, this);
    }
    else
      image_sub_ = nh.subscribe("usb_cam/image_raw", 1, &ColorCalibrator::topicImageCallback, this);
  }

  void cvImageCallback(const ros::TimerEvent& event)
  {
    cap_ >> frame_;
    processImage_();
  }

  void topicImageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      frame_ = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_FATAL("cv_bridge exception: %s", e.what());
      ros::shutdown();
    }

    processImage_();
  }

  void processImage_()
  {
    drawMask();
    cv::imshow("frame", frame_);
    cv::waitKey(3);
  }

  void drawMask()
  {
    cv::Mat mask, hsv;
    cv::cvtColor(frame_, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, lowerb_, upperb_, mask);
    cv::bitwise_and(frame_, cv::Scalar(0), frame_, mask); // to see the result
  }

  void reconfigureCallback(object_detector::ColorBoundsConfig &config, uint32_t level)
  {
    lowerb_ = cv::Vec<short,3>(config.lower_bound_hue, config.lower_bound_sat, config.lower_bound_val);
    upperb_ = cv::Vec<short,3>(config.upper_bound_hue, config.upper_bound_sat, config.upper_bound_val);
  }


private:
  void initializeCamera_(std::string capture_source)
  {
    try
    {
      cap_.open(std::stoi(capture_source));
    }
    catch (std::exception& e)
    {
      cap_.open(capture_source);
    }

    if (!cap_.isOpened())
    {
      std::string e = "Failed to load video";
      ROS_FATAL("%s", e.c_str());
      throw e;
    }
  }

  ros::NodeHandle nh_;
  ros::Timer timer_;
  cv::Mat frame_;

  cv::VideoCapture cap_;
  ros::Subscriber image_sub_;
  dynamic_reconfigure::Server<object_detector::ColorBoundsConfig> server_;

  cv::Vec<short, 3> lowerb_;
  cv::Vec<short, 3> upperb_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "color_calibrator");
  ros::NodeHandle nh;
  ColorCalibrator c(nh);
  ros::spin();
  return 0;
}
