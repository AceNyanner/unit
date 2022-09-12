#include "../src/briker.h"

namespace unit
{
    car::car()
    :   SPIN_RATE(30)
    {
        ros::NodeHandle nh(ros::this_node::getName());
        boost::thread point_thread=boost::thread(boost::bind(&car::pointThread,this));
        walk_pub_=nh.advertise<briker_strategy::WalkingParam>("walkParam",100);
        arm_pub_=nh.advertise<std_msgs::Int16>("armMode",100);
        command_pub_=nh.advertise<std_msgs::Int16>("command",1);
        command_pub_2=nh.advertise<std_msgs::Int16>("command2",1);
        startMode();
        //boost::thread process_thread=boost::thread(boost::bind(&car::processThread,this));
        boost::thread distance_thread=boost::thread(boost::bind(&car::distanceThread,this));
    }

    car::~car(){}

    int car::fabs(int a)
    {
        if(a>=0)
            return a;
        else
            return -a;
    }

    void car::pointCallback(const std_msgs::StringConstPtr& msg)
    {
        ROS_INFO("%s",msg->data.c_str());
        std::stringstream ss;
        ss<<msg->data.c_str();
        ss>>x1>>y1>>x2>>y2;
    }

    void car::pointThread()
    {
        ROS_INFO("point get");
        ros::NodeHandle nh(ros::this_node::getName());
        position_sub_=nh.subscribe("pointParam",1000,&car::pointCallback,this);
    }

    void car::process()
    {
        startMode();

        if(process_mode==SEARCH)
            search();
        else if(process_mode==WALK_FAR)
            walkFar();
        else if(process_mode==WALK_ACCURATE)
            walkAccurate();
        else if(process_mode==ADJUSTTOWARD)
            adjustToward();
        else if(process_mode==DOWNARM)
            downArm();
        else if(process_mode==LIFTARM)
            liftArm();
        else if(process_mode==MYCATCH)
            myCatch();
        else if(process_mode==TURN)
            turn();
        else if(process_mode==STOP)
            stop();
    }

    /*void car::processThread()
    {
        ros::Rate loop_rate(SPIN_RATE);

        while(ros::ok())
        {
            process();
            loop_rate.sleep();
        }
    }
    */

    void car::distanceCallback(const std_msgs::StringConstPtr& msg)
    {
        ROS_INFO("%s",msg->data.c_str());
        std::stringstream s;
        s<<msg->data.c_str();
        s>>dis;
    }

    void car::distanceThread()
    {
        ROS_INFO("distance get");
        ros::NodeHandle nh(ros::this_node::getName());
        distance_sub_=nh.subscribe("distanceParam",1000,&car::distanceCallback,this);
    }


     void car::startMode()
    {
        ros::NodeHandle nh(ros::this_node::getName());
        process_mode=SEARCH;
        premode=SEARCH;
        ROS_INFO("start!");
        command.data=1;
        command_pub_.publish(command);
        command2.data=2;
        command_pub_2.publish(command2);
    }

    void car::search()
    {
        setWalkingParam(0,0,0,0,0,0);
        if(x1!=0&&y1!=0&&x2!=0&&y2!=0)
        {
            premode=SEARCH;
            process_mode=WALK_FAR;
        }
        else if(x1==0&&y1==0&&x2==0&&y2==0)
        {
            premode=SEARCH;
            process_mode=TURN;
        }
        else if(premode==WALK_FAR)
        {
            if(x1!=0&&y1!=0&&x2!=0&&y2!=0)
            {
                premode=SEARCH;
                process_mode=WALK_FAR;
            }   
            else if(x1==0&&y1==0&&x2==0&&y2==0)
            {
                premode=SEARCH;
                process_mode=TURN;
            }
            else if((fabs(x1-x2)<105)&&(fabs(x1-x2)>95))//first judge whether need to change walk params
            {
                premode=WALK_FAR;
                process_mode=WALK_ACCURATE;
            }
        }

    }

    void car::adjustToward()
    {
        setWalkingParam(0,0,0,0,0,0);
        if(fabs((x1+x2)/2-200)<=3)//zhengdui
        {
            premode=ADJUSTTOWARD;
            process_mode=DOWNARM;
        }
        else if((200-(x1+x2)/2)>3)//right swift
        {
            setWalkingParam(200-(x1+x2)/2,0,0,10,0,0);//first param
            usleep(300*1000);
            premode=ADJUSTTOWARD;
            process_mode=DOWNARM;
        }
        else if(((x1+x2)/2-200)>3)//left swift
        {
            setWalkingParam(-((x1+x2)/2-200),0,0,10,0,0);//first param
            usleep(300*1000);
            premode=ADJUSTTOWARD;
            process_mode=DOWNARM;
        }
    }

    void car::walkFar()
    {
        setWalkingParam(0,20,0,0,20,0);
        usleep(300*1000);
        premode=WALK_FAR;
        process_mode=SEARCH;
    }

    void car::walkAccurate()
    {
        setWalkingParam(0,dis-15,0,0,10,0);
        usleep(300*1000);
        premode=WALK_ACCURATE;
        process_mode=ADJUSTTOWARD;
    }

    void car::downArm()
    {
        command3.data=3;
        command_pub_3.publish(command3);
        setArmMode(_DOWN);
        usleep(300*1000);
        premode=DOWNARM;
        process_mode=MYCATCH;
    }

    void car::myCatch()
    {
        setArmMode(_CATCH);
        usleep(300*1000);
        premode=MYCATCH;
        process_mode=LIFTARM;
    }
    
    void car::liftArm()
    {
        setArmMode(_UP);
        usleep(300*1000);
        premode=LIFTARM;
        process_mode=STOP;
    }

    void car::stop()
    {
        ROS_INFO("mission completed");
    }

    void car::turn()
    {
        if(premode==SEARCH)
        {
            setWalkingParam(0,0,45,0,0,20);
            usleep(500*1000);
            premode=TURN;
            process_mode=SEARCH;
        }
    }

    void car::setWalkingParam(int x_move,int y_move,int angle,int x_speed,int y_speed,int angle_speed)
    {
        next_move.x_move=x_move;
        next_move.y_move=y_move;
        next_move.angle=angle;
        next_move.x_speed=x_speed;
        next_move.y_speed=y_speed;
        next_move.angle_speed=angle_speed;
        walk_pub_.publish(next_move);
    }

    void car::setArmMode(int mode)
    {
        next_mode.data=mode;
        arm_pub_.publish(next_mode);
    }
} 
