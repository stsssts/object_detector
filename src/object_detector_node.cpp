#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "object_detector.cpp"


class ObjectDetectorNode
{
public:
  ObjectDetectorNode()
  {
    ros::NodeHandle node;

    detector_.setParameters(getParameters());

    image_sub_ = node.subscribe("usb_cam/image_raw", 1, &ObjectDetectorNode::imageCallback, this);

    line_pose_pub_ = node.advertise<geometry_msgs::TwistStamped>("line_pose", 1);
    basket_pose_pub_ = node.advertise<geometry_msgs::TwistStamped>("basket_pose", 1);
  }

  Parameters getParameters()
  {
    ros::NodeHandle node_priv("~");
    Parameters p;
    node_priv.param<double>("scan_width", p.scan_width, 4);
    node_priv.param<int>("image_width", p.image_width, 640);
    node_priv.param<int>("image_height", p.image_height, 480);
    node_priv.param<bool>("rotated", p.rotated, false);

    std::vector<int> lb, ub;
    node_priv.param<std::vector<int>>("line_detector/lower_bound", lb, {90, 90, 0});
    node_priv.param<std::vector<int>>("line_detector/upper_bound", ub, {115, 255, 255});
    p.line_bounds = ColorBounds(cv::Scalar(lb[0], lb[1], lb[2]),
                                cv::Scalar(ub[0], ub[1], ub[2]));
    ROS_INFO("Line detector parameters:");
    ROS_INFO("Lower bounds: [%d %d %d]", lb[0], lb[1], lb[2]);
    ROS_INFO("Upper bounds: [%d %d %d]", ub[0], ub[1], ub[2]);

    node_priv.param<std::vector<int>>("basket_detector/lower_bound", lb, {40, 0, 100});
    node_priv.param<std::vector<int>>("basket_detector/upper_bound", ub, {100, 255, 160});
    p.basket_bounds = ColorBounds(cv::Scalar(lb[0], lb[1], lb[2]),
                                  cv::Scalar(ub[0], ub[1], ub[2]));
    ROS_INFO("Basket detector parameters:");
    ROS_INFO("Lower bounds: [%d %d %d]", lb[0], lb[1], lb[2]);
    ROS_INFO("Upper bounds: [%d %d %d]", ub[0], ub[1], ub[2]);

    return p;
  }

  void publishLinePose(Line &l)
  {
    line_msg_.header.stamp = ros::Time::now();
    line_msg_.twist.linear.y = l.center;
    line_msg_.twist.angular.z = l.angle;
    line_pose_pub_.publish(line_msg_);
  }

  void publishBasketPose(cv::Point2f point)
  {
    basket_msg_.header.stamp = ros::Time::now();
    basket_msg_.twist.linear.x = point.x;
    basket_msg_.twist.linear.y = point.y;
    basket_pose_pub_.publish(basket_msg_);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    try
    {
      cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::cvtColor(cv_ptr_->image, imageHSV_, cv::COLOR_BGR2HSV);

    Line l;
    if (detector_.detectLine(imageHSV_, l))
      publishLinePose(l);

    cv::Point2f b;
    if (detector_.detectBasket(imageHSV_, b))
      publishBasketPose(b);
  }

private:
  ObjectDetector detector_;
  ros::Subscriber image_sub_;
  cv_bridge::CvImagePtr cv_ptr_;
  cv::Mat imageHSV_;

  ros::Publisher line_pose_pub_;
  ros::Publisher basket_pose_pub_;
  geometry_msgs::TwistStamped line_msg_;
  geometry_msgs::TwistStamped basket_msg_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detector");
  ObjectDetectorNode od;
  ros::spin();
  return 0;
}
