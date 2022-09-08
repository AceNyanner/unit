#include "lineStrategy.h"

namespace line {

void lineStrategy::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    img_height = msg->height;
    img_width = msg->width;
    ROS_INFO("img_height: %d\nimg_width: %d",img_height,img_width);
    return;
}

void lineStrategy::leftCallback(const line_vision::Line::ConstPtr& msg)
{
    left.clear();
    Point tmp;
    tmp.x = msg->topPoint.x;
    tmp.y = msg->topPoint.y;
    left.push_back(tmp);//先压入上面的点，后压入下面的点，遍历时上面的点在前，下面的点在后
    tmp.x = msg->bottomPoint.x;
    tmp.y = msg->bottomPoint.y;
    left.push_back(tmp);
    ROS_INFO("leftCallback running");
    return;
}

void lineStrategy::rightCallback(const line_vision::Line::ConstPtr& msg)
{
    right.clear();
    Point tmp;
    tmp.x = msg->topPoint.x;
    tmp.y = msg->topPoint.y;
    right.push_back(tmp);//先压入上面的点，后压入下面的点，遍历时上面的点在前，下面的点在后
    tmp.x = msg->bottomPoint.x;
    tmp.y = msg->bottomPoint.y;
    right.push_back(tmp);
    ROS_INFO("rightCallback running");
    return;
}

void lineStrategy::process()
{

    // testINFO();
    // ROS_INFO("process running");
    return;
}

void lineStrategy::adjustDirection()
{
    
}

void lineStrategy::testINFO()
{
    for (size_t i = 0; i < left.size(); i++)
    {
        // ROS_INFO("leftsize:%d",int(left.size()));
        ROS_INFO("left x:%d  y:%d",left[i].x,left[i].y);       
    }
    for (size_t i = 0; i < right.size(); i++)
    {
        ROS_INFO("right x:%d  y:%d",right[i].x,right[i].y);
    }
    // ROS_INFO("test running");
    return;
    
}

lineStrategy::lineStrategy():
    nh_(ros::this_node::getName()),
    it_(this->nh_)
{   
    img_sub_ = it_.subscribe("/line_vision_node/vision/line/image_output",1,&lineStrategy::imageCallback,this);
    line_left_sub_=nh_.subscribe<line_vision::Line>("/line_vision_node/vision/line/leftline",1,&lineStrategy::leftCallback,this);
    line_right_sub_=nh_.subscribe<line_vision::Line>("/line_vision_node/vision/line/rightline",1,&lineStrategy::rightCallback,this);
    
}



lineStrategy::~lineStrategy()
{

}
}