#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
//#include <cv_camera/capture.h>
#include <opencv2/opencv.hpp>

#include "object_detector.cpp"


class ObjectDetectorNode
{
public:
  ObjectDetectorNode()
  {

    ros::NodeHandle node;

    detector_.setParameters(getParameters());

    line_pose_pub_ = node.advertise<geometry_msgs::TwistStamped>("line_pose", 1);
    basket_pose_pub_ = node.advertise<geometry_msgs::TwistStamped>("basket_pose", 1);
  }

  Parameters getParameters()
  {
    ros::NodeHandle node_priv("~");
    Parameters p;
    node_priv.param<double>("scan_width", p.scan_width, 4);
    node_priv.param<int>("image_width", p.image_width, 320);
    node_priv.param<int>("image_height", p.image_height, 240);
    node_priv.param<bool>("rotated", p.rotated, false);

    int id;
    node_priv.param<int>("camera_id", id, 0);

    cap_.open(id);
    if (!cap_.isOpened())
    {
      std::stringstream stream;
      stream << "Camera " << id << " cannot be opened";
//      throw cv_camera::DeviceError(stream.str());
    }

    cap_.set(CV_CAP_PROP_FRAME_WIDTH, p.image_width);
    cap_.set(CV_CAP_PROP_FRAME_HEIGHT, p.image_height);

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


  void getImage()
  {
    cap_ >> frame_;
    cv::cvtColor(frame_, imageHSV_, cv::COLOR_BGR2HSV);

    Line l;
    if (detector_.detectLine(imageHSV_, l))
      publishLinePose(l);

//    cv::Point2f b;
//    if (detector_.detectBasket(imageHSV_, b))
//      publishBasketPose(b);
  }

private:
  cv::Mat frame_;
  cv::Mat imageHSV_;

  cv::VideoCapture cap_;

  ObjectDetector detector_;

  ros::Publisher line_pose_pub_;
  ros::Publisher basket_pose_pub_;

  geometry_msgs::TwistStamped line_msg_;
  geometry_msgs::TwistStamped basket_msg_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detector");
  ObjectDetectorNode od;
  ros::Rate loop_rate(15);
  while(ros::ok())
  {
    od.getImage();
    loop_rate.sleep();
  }
  return 0;
}
