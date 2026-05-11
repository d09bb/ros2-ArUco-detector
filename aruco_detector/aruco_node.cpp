#include <memory>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"

#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


class ArucoDetectorNode : public rclcpp::Node
{
public:
    ArucoDetectorNode() : Node("aruco_detector")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&ArucoDetectorNode::image_callback, this, std::placeholders::_1)
        );

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco/image", 10);
        id_pub_ = this->create_publisher<std_msgs::msg::Int32>("/aruco/id", 10);
        center_x_pub_ = this->create_publisher<std_msgs::msg::Int32>("/aruco/center_x", 10);
        center_y_pub_ = this->create_publisher<std_msgs::msg::Int32>("/aruco/center_y", 10);
        error_x_pub_ = this->create_publisher<std_msgs::msg::Int32>("/aruco/error_x", 10);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        detector_params_ = cv::aruco::DetectorParameters::create();

        RCLCPP_INFO(this->get_logger(), "Aruco detector node started.");
        RCLCPP_INFO(this->get_logger(), "Subscribing: /camera/color/image_raw");
        RCLCPP_INFO(this->get_logger(), "Publishing: /aruco/image, /aruco/id, /aruco/center_x, /aruco/center_y, /aruco/error_x");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;

        try
        {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty image frame received.");
            return;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<std::vector<cv::Point2f>> rejected;

        cv::aruco::detectMarkers(
            gray,
            dictionary_,
            corners,
            ids,
            detector_params_,
            rejected
        );

        int image_center_x = frame.cols / 2;

        if (!ids.empty())
        {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);

            int selected_index = 0;
            double max_area = 0.0;

            for (size_t i = 0; i < corners.size(); i++)
            {
                double area = cv::contourArea(corners[i]);
                if (area > max_area)
                {
                    max_area = area;
                    selected_index = static_cast<int>(i);
                }
            }

            const auto &selected_corners = corners[selected_index];

            int cx = 0;
            int cy = 0;

            for (const auto &p : selected_corners)
            {
                cx += static_cast<int>(p.x);
                cy += static_cast<int>(p.y);
            }

            cx /= 4;
            cy /= 4;

            int error_x = cx - image_center_x;
            int marker_id = ids[selected_index];

            cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1);
            cv::line(
                frame,
                cv::Point(image_center_x, 0),
                cv::Point(image_center_x, frame.rows),
                cv::Scalar(255, 0, 0),
                2
            );

            cv::putText(
                frame,
                "ID: " + std::to_string(marker_id),
                cv::Point(cx + 10, cy - 10),
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                cv::Scalar(0, 255, 0),
                2
            );

            std_msgs::msg::Int32 id_msg;
            std_msgs::msg::Int32 center_x_msg;
            std_msgs::msg::Int32 center_y_msg;
            std_msgs::msg::Int32 error_x_msg;

            id_msg.data = marker_id;
            center_x_msg.data = cx;
            center_y_msg.data = cy;
            error_x_msg.data = error_x;

            id_pub_->publish(id_msg);
            center_x_pub_->publish(center_x_msg);
            center_y_pub_->publish(center_y_msg);
            error_x_pub_->publish(error_x_msg);

            RCLCPP_INFO(
                this->get_logger(),
                "id=%d, cx=%d, cy=%d, error_x=%d",
                marker_id,
                cx,
                cy,
                error_x
            );
        }
        else
        {
            std_msgs::msg::Int32 id_msg;
            std_msgs::msg::Int32 center_x_msg;
            std_msgs::msg::Int32 center_y_msg;
            std_msgs::msg::Int32 error_x_msg;

            id_msg.data = -1;
            center_x_msg.data = -1;
            center_y_msg.data = -1;
            error_x_msg.data = 0;

            id_pub_->publish(id_msg);
            center_x_pub_->publish(center_x_msg);
            center_y_pub_->publish(center_y_msg);
            error_x_pub_->publish(error_x_msg);
        }

        auto out_msg = cv_bridge::CvImage(
            msg->header,
            "bgr8",
            frame
        ).toImageMsg();

        image_pub_->publish(*out_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr id_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr center_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr center_y_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr error_x_pub_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
