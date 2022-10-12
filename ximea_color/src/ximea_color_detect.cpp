#include <ros/ros.h>
#include <iostream>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>
#include <robot_msgs/ColorWithID.h>
#include <robot_msgs/ColorWithIDArray.h>
#include <robot_msgs/LuggageColor.h>
#include <robot_msgs/LuggageColorArray.h>
#include <opencv2/core.hpp> 
#include <opencv2/opencv.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <math.h>

bool flag = false;
robot_msgs::ColorWithIDArray colorWithIDArray;

void callback(const robot_msgs::ColorWithIDArray::ConstPtr& colorWithIDArray_) {
    colorWithIDArray.colors = colorWithIDArray_->colors;
    flag = true;
}

double dist(std::vector<double> c1, std::vector<double> c2) {
    return sqrt(pow(c1[0] - c2[0], 2) + pow(c1[1] - c2[1], 2) + pow(c1[2] - c2[2], 2)); 
}

std::vector<double> get_coords(std_msgs::ColorRGBA bgr_test) {
    cv::Mat cv_bgr(cv::Size(1, 1), CV_8UC3);
    cv_bgr.at<cv::Vec<uint8_t, 3>>(0, 0) = cv::Vec<uint8_t, 3>(bgr_test.b, bgr_test.g, bgr_test.r);
    cv::Mat cv_hsv;
    cv::cvtColor(cv_bgr, cv_hsv, cv::COLOR_BGR2HSV);
    double h = 2 * M_PI * cv_hsv.at<cv::Vec<uint8_t, 3>>(0, 0)[0] / 180.0;
    
    double s = cv_hsv.at<cv::Vec<uint8_t, 3>>(0, 0)[1] / 255.0;
    double v = cv_hsv.at<cv::Vec<uint8_t, 3>>(0, 0)[2] / 255.0;
    double x = v * s * cos(h);
    double y = v * s * sin(h);
    double z = v;
    std::vector<double> array = {x, y, z};
    return array;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "color_detect");
    ros::NodeHandle nh_sub;
    ros::NodeHandle nh_pub;
    ros::Subscriber sub = nh_sub.subscribe("/test_color", 100, callback);
    ros::Publisher pub = nh_sub.advertise<robot_msgs::LuggageColorArray>("/luggage_info", 10);

    std_msgs::ColorRGBA red_bgr;
    red_bgr.r = 255;
    red_bgr.g = 0;
    red_bgr.b = 0;

    std_msgs::ColorRGBA green_bgr;
    green_bgr.r = 0;
    green_bgr.g = 255;
    green_bgr.b = 0;

    std_msgs::ColorRGBA blue_bgr;
    blue_bgr.r = 0;
    blue_bgr.g = 0;
    blue_bgr.b = 255;
    
    std_msgs::ColorRGBA yellow_bgr;
    yellow_bgr.r = 255;
    yellow_bgr.g = 255;
    yellow_bgr.b = 0;

    std::vector<double> red_coords = get_coords(red_bgr);
    std::vector<double> green_coords = get_coords(green_bgr);
    std::vector<double> blue_coords = get_coords(blue_bgr);
    std::vector<double> yellow_coords = get_coords(yellow_bgr);

    while (ros::ok()) {

        if (flag) {

            robot_msgs::LuggageColorArray lca;
            lca.colors.clear();
            for (const auto& colorWithID : colorWithIDArray.colors) {
                std::vector<double> test_coords = get_coords(colorWithID.color);
                double red_dist = dist(test_coords, red_coords);
                double green_dist = dist(test_coords, green_coords);
                double blue_dist = dist(test_coords, blue_coords);
                double yellow_dist = dist(test_coords, yellow_coords);
                double dists[4] = {
                    red_dist, green_dist, blue_dist, yellow_dist
                };
                int idx = -1;
                double min_val = std::numeric_limits<double>::max();
                for (int i = 0; i < 4; i++) {
                    if (dists[i] < min_val) {
                        min_val = dists[i];
                        idx = i;
                    }
                }

                // 0: red
                // 1: green
                // 2: blue
                // 3: yellow
                robot_msgs::LuggageColor luggageColor;
                luggageColor.fiducial_id = colorWithID.fiducial_id;
                luggageColor.color_code = idx;

                lca.colors.push_back(luggageColor);

            }

            lca.header.stamp = ros::Time::now();            
            pub.publish(lca);
            flag = false;
        }
        ros::spinOnce();
    }
}