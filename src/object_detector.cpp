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

class ColorDetector
{
public:
  void setBounds(ColorBounds b)
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

class LineDetector
{
public:
  LineDetector() {}

  void setParameters(cv::Size img_size, int scan_width, ColorBounds bounds)
  {
    cd_.setBounds(bounds);

    int w = img_size.width, h = img_size.height, sw = scan_width;
    rois_.push_back(cv::Rect(cv::Point(0, 0),     cv::Point(w, sw)));
    rois_.push_back(cv::Rect(cv::Point(0, h-sw),  cv::Point(w, h)));
    rois_.push_back(cv::Rect(cv::Point(0, sw),    cv::Point(sw, h-sw)));
    rois_.push_back(cv::Rect(cv::Point(w-sw, sw), cv::Point(w, h-sw)));
  }

  bool detect(cv::Mat& imgHSV, cv::Vec4f& line)
  {
    std::vector<cv::Point> points;
    cv::Point p;
    for (auto& roi : rois_)
    {
      if (cd_.getPixel(imgHSV(roi), p))
        points.push_back(p + roi.tl());
    }

    if (points.size() < 2)
      return false;

    cv::fitLine(points, line, CV_DIST_L2, 0, 0.01, 0.01);
    return true;
//    if (rotated_)
//    {
//      line.angle = atan2(l[0], l[1]) - M_PI_2;
//      line.center = (fabs(line.angle) < M_PI_4) ? (l[3] - l[1] * l[2] / l[0]) : 0;
//    }
//    else
//    {
//      line.angle = atan2(l[1], l[0]);
//      line.angle += (line.angle < 0) ? M_PI_2 : -M_PI_2;
//      line.center = (fabs(line.angle) < M_PI_4) ? (l[2] - l[0] * l[3] / l[1]) : 0;
//    }
  }

private:
  ColorDetector cd_;
  std::vector<cv::Rect> rois_;
};

class BasketDetector
{
public:
  BasketDetector() {}

  void setParameters(ColorBounds bounds)
  {
    cd_.setBounds(bounds);
  }

  bool detect(cv::Mat& imgHSV, cv::Point& center)
  {
    if (cd_.getPixel(imgHSV, center))
      return true;

    return false;
  }

private:
  ColorDetector cd_;
};

class EllipseDetector
{
public:
  EllipseDetector() {}

  void setParameters() {}

  bool detect(cv::Mat& imgGray, cv::RotatedRect& ellipse)
  {
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> hull;

    cv::GaussianBlur(imgGray, imgGray, cv::Size(5, 5), 2);

    double cannyParams = cv::threshold(imgGray, imgGray, 0, 255, CV_THRESH_BINARY_INV + CV_THRESH_OTSU);
    cv::Canny(imgGray, imgGray, cannyParams, cannyParams / 2.0F);
    cv::findContours(imgGray, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    for (int i = 0; i < contours.size(); i++)
    {
      if (contours.at(i).size() < 5)
        continue;

      if (std::fabs(cv::contourArea(contours.at(i))) < 300.0)
        continue;

      cv::RotatedRect bEllipse = cv::fitEllipse(contours.at(i));
      cv::convexHull(contours.at(i), hull, true);
      cv::approxPolyDP(hull, hull, 15, true);
      if (!cv::isContourConvex(hull))
        continue;

      double area = cv::contourArea(contours.at(i));
      cv::Rect r = cv::boundingRect(contours.at(i));
      double radius = r.width / 2.0;
      if (std::abs(1.0 - ((double) r.width / (double) r.height)) <= 0.5 &&
          std::abs(1.0 - (area / (CV_PI * std::pow(radius, 2.0)))) <= 0.2)
      {
        ellipse = bEllipse;
        return true;
      }
    }
    return false;
  }
};
