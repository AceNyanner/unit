#ifndef LINESTRATEGY_H
#define LINESTRATEGY_H

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <boost/bind.hpp>
#include <vector>

#include "line_vision/Line.h"

using namespace std;
using namespace cv;

namespace line{
class lineStrategy
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_sub_;
    ros::Subscriber line_left_sub_;
    ros::Subscriber line_right_sub_;
    int img_width,img_height;
    vector<Point> left,right;
    //通过传入的数据得到的梯形上下边线的中点
    int m_top_center,m_bottom_center;
public:
    lineStrategy();
    ~lineStrategy();
    //主进程
    void process();
    //打印左右两条线的信息
    void testINFO();
    //判断车应该走的方向
    void adjustDirection();
    //
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rightCallback(const line_vision::Line::ConstPtr& msg);
    void leftCallback(const line_vision::Line::ConstPtr& msg);
};
}

#endif