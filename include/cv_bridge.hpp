#ifndef CV_BRIDGE_HPP
#define CV_BRIDGE_HPP

#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>
#include <vector>

namespace cv_bridge {

class CvImage {
public:
    std::string encoding;
    cv::Mat image;

    typedef std::shared_ptr<CvImage> Ptr;
    typedef std::shared_ptr<const CvImage> ConstPtr;

    CvImage() {}
    CvImage(const std::string& encoding, const cv::Mat& image)
        : encoding(encoding), image(image) {}
};

inline CvImage::Ptr toCvCopy(const sensor_msgs::msg::Image::ConstSharedPtr& source, const std::string& encoding = "bgr8") {
    CvImage::Ptr out = std::make_shared<CvImage>();
    out->encoding = encoding;

    int type;
    if (source->encoding == "mono8") {
        type = CV_8UC1;
    } else if (source->encoding == "bgr8") {
        type = CV_8UC3;
    } else if (source->encoding == "rgb8") {
        type = CV_8UC3;
    } else if (source->encoding == "bgra8") {
        type = CV_8UC4;
    } else if (source->encoding == "rgba8") {
        type = CV_8UC4;
    } else if (source->encoding == "mono16") {
        type = CV_16UC1;
    } else if (source->encoding == "32FC1") {
        type = CV_32FC1;
    } else {
        throw std::runtime_error("Unsupported image encoding: " + source->encoding);
    }

    cv::Mat raw(source->height, source->width, type, const_cast<uchar*>(&source->data[0]), source->step);
    
    if (encoding == "bgr8") {
        if (source->encoding == "rgb8") {
            cv::cvtColor(raw, out->image, cv::COLOR_RGB2BGR);
        } else if (source->encoding == "mono8") {
            cv::cvtColor(raw, out->image, cv::COLOR_GRAY2BGR);
        } else if (source->encoding == "bgr8") {
            out->image = raw.clone();
        } else {
            throw std::runtime_error("Conversion to bgr8 from " + source->encoding + " not implemented");
        }
    } else if (encoding == "mono8") {
        if (source->encoding == "mono8") {
            out->image = raw.clone();
        } else if (source->encoding == "bgr8") {
            cv::cvtColor(raw, out->image, cv::COLOR_BGR2GRAY);
        } else if (source->encoding == "rgb8") {
            cv::cvtColor(raw, out->image, cv::COLOR_RGB2GRAY);
        } else {
            throw std::runtime_error("Conversion to mono8 from " + source->encoding + " not implemented");
        }
    } else {
        out->image = raw.clone();
    }
    
    return out;
}

} // namespace cv_bridge

#endif // CV_BRIDGE_HPP
