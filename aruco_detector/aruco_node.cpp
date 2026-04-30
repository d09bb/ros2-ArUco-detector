#include <memory>
#include <vector>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

class ArucoDetectorNode : public rclcpp::Node
{
public:
    ArucoDetectorNode() : Node("aruco_detector")
    {
        id_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/aruco/id", 10);
        center_x_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/aruco/center_x", 10);
        center_y_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/aruco/center_y", 10);
        error_x_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/aruco/error_x", 10);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw",
            qos,
            std::bind(&ArucoDetectorNode::image_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/aruco/image",
            qos
        );

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        detector_ = std::make_unique<cv::aruco::ArucoDetector>(
            dictionary_,
            detector_params_,
            cv::aruco::RefineParameters()
        );
        RCLCPP_INFO(this->get_logger(), "Aruco Detector Node Started");
    }

    ~ArucoDetectorNode()
    {
        cv::destroyAllWindows();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "PUBLISHING NOW");

        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (const cv_bridge::Exception & e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> rejected;

        detector_->detectMarkers(gray, corners, ids, rejected);

        if (!ids.empty())
        {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);

            std::ostringstream oss;
            oss << "Detected IDs: ";
            for (size_t i = 0; i < ids.size(); ++i)
            {
                oss << ids[i];
                if (i + 1 < ids.size())
                    oss << ", ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
            
            for (size_t i = 0; i < ids.size(); ++i)
            {
                float center_x = 0.0f;
                float center_y = 0.0f;

                for (const auto & p : corners[i])
                {
                    center_x += p.x;
                    center_y += p.y;
                }

                center_x /= 4.0f;
                center_y /= 4.0f;

                float image_center_x = frame.cols / 2.0f;
                float error_x = center_x - image_center_x;

                std_msgs::msg::Int32 id_msg;
                std_msgs::msg::Float32 center_x_msg;
                std_msgs::msg::Float32 center_y_msg;
                std_msgs::msg::Float32 error_x_msg;

                id_msg.data = ids[i];
                center_x_msg.data = center_x;
                center_y_msg.data = center_y;
                error_x_msg.data = error_x;

                id_publisher_->publish(id_msg);
                center_x_publisher_->publish(center_x_msg);
                center_y_publisher_->publish(center_y_msg);
                error_x_publisher_->publish(error_x_msg);

                RCLCPP_INFO(
                    this->get_logger(),
                    "ID=%d, center_x=%.1f, center_y=%.1f, error_x=%.1f",
                    ids[i], center_x, center_y, error_x
                );
            }

        }

        cv::imshow("Aruco Detection", frame);
        cv::waitKey(1);

        auto output_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        publisher_->publish(*output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr id_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr center_x_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr center_y_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_x_publisher_;

    cv::aruco::Dictionary dictionary_;
    cv::aruco::DetectorParameters detector_params_;
    std::unique_ptr<cv::aruco::ArucoDetector> detector_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}