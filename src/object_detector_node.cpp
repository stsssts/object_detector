#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <akara_msgs/Ellipse.h>
#include <akara_msgs/Line.h>
#include <akara_msgs/Basket.h>

#include <std_srvs/SetBool.h>

#include "object_detector.cpp"


class ObjectDetectorNode
{
public:
  ObjectDetectorNode(ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh("~");

    int fps;
    cv::Size img_size;
    bool use_topic;
    std::string cam_id;

    pnh.param("camera_id", cam_id, std::string("0"));
    pnh.param("fps", fps, 10);
    pnh.param("width", img_size.width, 320);
    pnh.param("height", img_size.height, 240);
    pnh.param("use_topic", use_topic, false);
    pnh.param("enabled", enabled_, true);
    pnh.param("debug", debug_, true);

    if (pnh.hasParam("ellipse_detector"))
    {
      detect_ellipse_ = true;
      ROS_INFO("Initialized ellipse detector");
      ellipse_pub_ = nh.advertise<akara_msgs::Ellipse>("ellipse", 1);
    }

    if (pnh.hasParam("line_detector"))
    {
      detect_line_ = true;
      int scan_width;
      std::vector<int> lb, ub;
      pnh.param<int>("line_detector/scan_width", scan_width, 4);
      pnh.param< std::vector<int> >("line_detector/lower_bound", lb, {90, 90, 0});
      pnh.param< std::vector<int> >("line_detector/upper_bound", ub, {115, 255, 255});
      ColorBounds cb(cv::Scalar(lb[0], lb[1], lb[2]), cv::Scalar(ub[0], ub[1], ub[2]));
      ROS_INFO("Initialized line detector, lower bound: (%d %d %d), upper bound: (%d %d %d)",
               lb[0], lb[1], lb[2], ub[0], ub[1], ub[2]);
      ROS_INFO("scan w: %i", scan_width);

      line_detector_.setParameters(img_size, scan_width, cb);
      line_pub_ = nh.advertise<akara_msgs::Line>("line", 1);
    }

    if (pnh.hasParam("basket_detector"))
    {
      detect_basket_ = true;
      std::vector<int> lb, ub;
      pnh.param< std::vector<int> >("basket_detector/lower_bound", lb, {90, 90, 0});
      pnh.param< std::vector<int> >("basket_detector/upper_bound", ub, {115, 255, 255});
      ColorBounds cb(cv::Scalar(lb[0], lb[1], lb[2]), cv::Scalar(ub[0], ub[1], ub[2]));
      ROS_INFO("Initialized basket detector");
      basket_detector_.setParameters(cb);
      basket_pub_ = nh.advertise<akara_msgs::Basket>("basket", 1);
    }

    enable_srv_ = pnh.advertiseService("enable", &ObjectDetectorNode::enableCallback, this);

    if (!use_topic)
    {
      initializeCamera_(cam_id, img_size, fps);
      timer_ = nh.createTimer(ros::Rate(fps), &ObjectDetectorNode::cvImageCallback, this);
    }
    else
      image_sub_ = nh.subscribe("usb_cam/image_raw", 1, &ObjectDetectorNode::topicImageCallback, this);
  }

  void cvImageCallback(const ros::TimerEvent& event)
  {
    if (!enabled_)
      return;

    cap_ >> frame_;
    processImage_();
  }

  void topicImageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    if (!enabled_)
      return;

    cv_bridge::CvImagePtr cv_ptr_;
    try
    {
      cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      frame_ = cv_ptr_->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    processImage_();
  }

  bool enableCallback(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
  {
    enabled_ = req.data;
    return true;
  }


private:
  void initializeCamera_(std::string capture_source, cv::Size size, int fps)
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

    if (cap_.set(CV_CAP_PROP_FRAME_WIDTH, size.width))
      ROS_INFO("Set camera width to %i", size.width);

    if (cap_.set(CV_CAP_PROP_FRAME_HEIGHT, size.height))
      ROS_INFO("Set camera height to %i", size.height);

    if (cap_.set(CV_CAP_PROP_FPS, fps))
      ROS_INFO("Set camera frame rate to %i", fps);
  }

  void processImage_()
  {
    if (detect_line_ || detect_basket_)
    {
      cv::Mat imgHSV;
      cv::cvtColor(frame_, imgHSV, cv::COLOR_BGR2HSV);

      detectLine_(imgHSV);
      detectBasket_(imgHSV);
    }

    if (detect_ellipse_)
    {
      cv::Mat imgGray;
      detectEllipse_(imgGray);
    }

    if (debug_)
    {
      cv::imshow("img", frame_);
      cv::waitKey(30);
    }
  }

  void detectLine_(cv::Mat& imgHSV)
  {
    if (!detect_line_)
      return;

    cv::Vec4f line;
    if (line_detector_.detect(imgHSV, line))
    {
      publishLine_(line);

      if (debug_)
        cv::line(frame_, cv::Point(line[2], line[3]),
            cv::Point(line[2] + 100 * line[0], line[3] + 100 * line[1]), cv::Scalar(0, 0, 255), 10);
    }
  }

  void detectBasket_(cv::Mat& imgHSV)
  {
    if (!detect_basket_)
      return;

    cv::Point center;
    if (basket_detector_.detect(imgHSV, center))
    {
      publishBasket_(center);

      if (debug_)
        cv::circle(frame_, center, 100, cv::Scalar(0, 255, 0), 10);
    }
  }

  void detectEllipse_(cv::Mat imgGray)
  {
    cv::RotatedRect ellipse;
    cv::cvtColor(frame_, imgGray, cv::COLOR_BGR2GRAY);
    if (ellipse_detector_.detect(imgGray, ellipse))
    {
      publishEllipse_(ellipse);
      if (debug_)
        cv::ellipse(frame_, ellipse, cv::Scalar(255,0,0), 10);
    }
  }

  void publishLine_(cv::Vec4f& line)
  {
    akara_msgs::Line msg;
    msg.header.stamp = ros::Time::now();
    msg.angle = atan2(line[1], line[0]);
    msg.angle += (msg.angle < 0) ? M_PI_2 : -M_PI_2;

    cv::Point2f p = pixelToPoint_(cv::Point(line[2], line[3]));
    msg.point.x = p.x;
    msg.point.y = p.y;

    line_pub_.publish(msg);
  }

  void publishBasket_(cv::Point& basket)
  {
    akara_msgs::Basket msg;
    msg.header.stamp = ros::Time::now();

    cv::Point2f p = pixelToPoint_(basket);
    msg.center.x = p.x;
    msg.center.y = p.y;

    basket_pub_.publish(msg);
  }

  void publishEllipse_(cv::RotatedRect& ellipse)
  {
    akara_msgs::Ellipse msg;
    msg.header.stamp = ros::Time::now();

    cv::Point2f center =  pixelToPoint_(ellipse.center);
    msg.center.x = center.x;
    msg.center.y = center.y;

    msg.size.width = ellipse.size.width;
    msg.size.height = ellipse.size.height;

    ellipse_pub_.publish(msg);
  }

  cv::Point2f pixelToPoint_(cv::Point pixel)
  {
    cv::Point2f p(pixel.x - frame_.cols/2.0,
                  pixel.y - frame_.rows/2.0);
    p.x /= frame_.cols / 2.0;
    p.y /= frame_.rows / 2.0;
    return p;
  }

  cv::VideoCapture cap_;
  ros::Timer timer_;
  cv::Mat frame_;

  bool enabled_;
  bool debug_;

  bool detect_ellipse_ = false;
  bool detect_basket_ = false;
  bool detect_line_ = false;

  EllipseDetector ellipse_detector_;
  LineDetector line_detector_;
  BasketDetector basket_detector_;

  ros::Subscriber image_sub_;

  ros::Publisher ellipse_pub_;
  ros::Publisher line_pub_;
  ros::Publisher basket_pub_;

  ros::ServiceServer enable_srv_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ellipse_detector");
  ros::NodeHandle nh;
  ObjectDetectorNode detector(nh);
  ros::spin();
  return 0;
}
