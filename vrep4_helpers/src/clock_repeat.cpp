
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <rosgraph_msgs/Clock.h>

ros::Publisher pub;
double last_sim_time = -1;
ros::WallTime last_print;

void timeCallback(std_msgs::Float32ConstPtr msg) {
    rosgraph_msgs::Clock clock;
    if (last_sim_time < 0) {
        last_sim_time = msg->data;
    }
    clock.clock.sec = int(msg->data);
    clock.clock.nsec = (msg->data - clock.clock.sec)*1e9;
    pub.publish(clock);
    ros::WallTime now = ros::WallTime::now();
    if ((now - last_print).toSec() > 5.0) {
        double ratio = 100.*(msg->data-last_sim_time)/(now-last_print).toSec();
        if (ratio > 1) {
            // Skipping, initial data
            ROS_INFO("Real-time ratio: %.0f%%",ratio);
        }
        last_print = now;
        last_sim_time = msg->data;
    }
}

int main(int argc, char * argv[]) {

    ros::init(argc,argv,"time_republish");
    last_print = ros::WallTime::now();
    ros::NodeHandle nh;
    pub = nh.advertise<rosgraph_msgs::Clock>("/clock",1);
    ros::Subscriber sub = nh.subscribe("/vrep/simTime",1,timeCallback);
    ros::spin();
    return 0;
}


