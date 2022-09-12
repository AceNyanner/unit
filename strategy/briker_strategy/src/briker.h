#ifndef UNIT_BRIKER_H_
#define UNIT_BRIKER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <boost/thread.hpp>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include "strategy.h"
#include "briker_strategy/WalkingParam.h"

#define _CATCH 1
#define _UP 2
#define _DOWN 3

namespace unit
{
    class car : public Strategy
    {
    public:
        enum State
        {
            SEARCH,
            WALK_FAR,
            WALK_ACCURATE,
            ADJUSTTOWARD,
            LIFTARM,
            DOWNARM,
            MYCATCH,
            STOP,
            TODESTINATION,
            TURN,
        };

        car();
        ~car();
    
    //protected:
        //boost::thread process_thread;
        //void processThread();
        void process();
        boost::thread point_thread;
        void pointThread();
        void pointCallback(const std_msgs::StringConstPtr& msg);
        boost::thread distance_thread;
        void distanceThread();
        void distanceCallback(const std_msgs::StringConstPtr& msg);

        void startMode();
        void search();
        void walkFar();
        void walkAccurate();
        void adjustToward();
        void downArm();
        void liftArm();
        void myCatch();
        void put();
        void turn();
        void stop();
        void setWalkingParam(int x_move,int y_move,int angle,int x_speed,int y_speed,int angle_spped);
        void setArmMode(int mode);
    
        ros::Subscriber position_sub_;
        ros::Subscriber distance_sub_;
        ros::Publisher walk_pub_;
        ros::Publisher arm_pub_;

        briker_strategy::WalkingParam next_move;
        std_msgs::Int16 next_mode;

        int process_mode,premode;
        int x1,y1,x2,y2,dis;
        int SPIN_RATE;
        
        int fabs(int a);
        
    };
}

#endif