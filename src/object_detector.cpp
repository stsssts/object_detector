#include <opencv2/imgproc/imgproc.hpp>

struct ColorBounds
{
  ColorBounds()
  {
    lower = cv::Scalar(0, 0, 0);
    upper = cv::Scalar(255, 255, 255);
  }

  ColorBounds(cv::Scalar l, cv::Scalar b)
  {
    lower = l;
    upper = b;
  }

  cv::Scalar lower;
  cv::Scalar upper;
};

struct Parameters
{
  double scan_width;
  int image_width;
  int image_height;
  bool rotated;

  ColorBounds line_bounds;
  ColorBounds basket_bounds;
};

struct Line
{
  double center;
  double angle;
};

class ColorDetector
{
public:
  void setParameters(ColorBounds b)
  {
    bounds_ = b;
  }

  bool getPixel(cv::Mat imgHSV, cv::Point &pixel)
  {
    cv::inRange(imgHSV, bounds_.lower, bounds_.upper, mask_);

    cv::Mat locations;
    cv::findNonZero(mask_, locations);
    if (!locations.empty() && locations.total() > imgHSV.total() * 0.05)
    {
      cv::Scalar mean = cv::mean(locations);
      pixel = cv::Point(mean[0], mean[1]);
      return true;
    }
    return false;
  }

private:
  cv::Mat mask_;
  ColorBounds bounds_;
};

class ObjectDetector
{
public:
  ObjectDetector() {}

  void setParameters(Parameters p)
  {
    params_ = p;
    line_cd_.setParameters(params_.line_bounds);
    basket_cd_.setParameters(params_.basket_bounds);

    int w = p.image_width, h = p.image_height, sw = p.scan_width;
    rois_.push_back(cv::Rect(cv::Point(0, 0),     cv::Point(w, sw)));
    rois_.push_back(cv::Rect(cv::Point(0, h-sw),  cv::Point(w, h)));
    rois_.push_back(cv::Rect(cv::Point(0, sw),    cv::Point(sw, h-sw)));
    rois_.push_back(cv::Rect(cv::Point(w-sw, sw), cv::Point(w, h-sw)));
  }

  bool detectLine(cv::Mat &imgHSV, Line &line)
  {
    std::vector<cv::Point2f> points;
    cv::Point p;
    for (auto& roi : rois_)
    {
      if (line_cd_.getPixel(imgHSV(roi), p))
        points.push_back(pixelToPoint(p + roi.tl()));
    }

    if (points.size() < 2)
      return false;

    cv::Vec4f l;
    cv::fitLine(points, l, CV_DIST_L2, 0, 0.01, 0.01);

    if (params_.rotated)
    {
      line.angle = atan2(l[0], l[1]) - M_PI_2;
      line.center = (fabs(line.angle) < M_PI_4) ? (l[3] - l[1] * l[2] / l[0]) : 0;
    }
    else
    {
      line.angle = atan2(l[1], l[0]);
      line.angle += (line.angle < 0) ? M_PI_2 : -M_PI_2;
      line.center = (fabs(line.angle) < M_PI_4) ? (l[2] - l[0] * l[3] / l[1]) : 0;
    }

    return true;
  }

  bool detectBasket(cv::Mat &imgHSV, cv::Point2f &basket)
  {
    cv::Point center;
    if (!basket_cd_.getPixel(imgHSV, center))
      return false;

    basket = pixelToPoint(center);
    return true;
  }

private:
  Parameters params_;

  std::vector<cv::Rect> rois_;

  ColorDetector line_cd_;
  ColorDetector basket_cd_;

  cv::Point2f pixelToPoint(cv::Point pixel)
  {
    cv::Point2f p(pixel.x - params_.image_width/2.0,
                  pixel.y - params_.image_height/2.0);
    p.x /= params_.image_width / 2.0;
    p.y /= params_.image_height / 2.0;
    return p;
  }
};
