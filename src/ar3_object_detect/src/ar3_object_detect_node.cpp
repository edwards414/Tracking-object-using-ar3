#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include "ar3_object_detect/PointArray.h"
#include <ar3_object_detect/GetCirclePositions.h>

using namespace cv;

image_transport::Publisher pub;
image_transport::Subscriber sub;
// ros::Publisher points_pub;
std::vector<geometry_msgs::Point> points;

bool getCirclePositions(ar3_object_detect::GetCirclePositions::Request &req,
                        ar3_object_detect::GetCirclePositions::Response &res) {
    res.positions = points;
    return true;
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat hsv, mask;

        // Convert the image to HSV color space
        cvtColor(image, hsv, COLOR_BGR2HSV);

        // Define the range of red color in HSV
        Scalar lower_red = Scalar(0, 120, 70);
        Scalar upper_red = Scalar(10, 255, 255);
        inRange(hsv, lower_red, upper_red, mask);

        // Additional range for red color
        Scalar lower_red_2 = Scalar(170, 120, 70);
        Scalar upper_red_2 = Scalar(180, 255, 255);
        Mat mask2;
        inRange(hsv, lower_red_2, upper_red_2, mask2);

        // Combine the masks
        mask = mask | mask2;

        // Reduce noise by using a Gaussian blur
        GaussianBlur(mask, mask, Size(9, 9), 2, 2);

        // Apply Hough Circle Transform
        std::vector<Vec3f> circles;
        HoughCircles(mask, circles, HOUGH_GRADIENT, 1, mask.rows / 8, 100, 30, 0, 0);
        points.clear();
        for (size_t i = 0; i < circles.size(); i++)
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // Draw the circle outline
            circle(image, center, radius, Scalar(0, 255, 0), 2);
            // Draw the circle center
            circle(image, center, 3, Scalar(0, 0, 255), 3);
            printf("radius %d\n" ,radius);
            geometry_msgs::Point point_msg;
            point_msg.x = center.x;
            point_msg.y = center.y;
            point_msg.z = 0.1;
            points.push_back(point_msg);
        }


        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        int thickness = 2;
        int baseline = 0;
        double fontScale = 1.0;
        std::string text = "detect" + std::to_string(circles.size()) + "circles";
        cv::Scalar color = cv::Scalar(255, 255, 255); // 白色
        cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
        cv::Point textOrg(image.cols - textSize.width - 10, textSize.height + 10);

        // 计算文本位置（右上角）
        putText(image, text, textOrg, fontFace, fontScale, color, thickness);

        // ar3_object_detect::PointArray points_msg;
        // points_msg.points = points;
        // points_pub.publish(points_msg);
        sensor_msgs::ImagePtr processed_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub.publish(processed_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    sub = it.subscribe("/ar3/camera1/image_raw", 1, imageCallback);
    pub = it.advertise("/image/processed", 1);
    // points_pub = nh.advertise<ar3_object_detect::PointArray>("/detected_circle_centers", 1); // 使用自定义消息类型
    ros::ServiceServer service = nh.advertiseService("get_circle_positions", getCirclePositions);

    ros::spin();
    return 0;
}
