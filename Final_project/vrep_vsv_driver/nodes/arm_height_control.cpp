#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <geometry_msgs/Twist.h>

class ArmHeightControl {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher twist_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        std::string camera_frame_;

        float min_z;
        double threshold_z;
        double gain;
        geometry_msgs::Twist v;
        pcl::PointCloud<pcl::PointXYZ> lastpc_;


    protected:

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(camera_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(camera_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            //
            unsigned int n = temp.size();
            min_z = 10;
            for (unsigned int i=0;i<n;i++) {
                float z = temp[i].z;
                if (abs(z) < 1e-2) {
                    // Bogus point, ignore ?
                    continue;
                }
                z = lastpc_[i].z;
                if (z < min_z){
                    min_z = z;
                }
            }
            ROS_INFO("Got min_z : %.2f", min_z);
            v.linear.z = gain * (min_z - threshold_z);
            twist_pub_.publish(v);
        }

    public:
        ArmHeightControl() : nh_("~") {
            nh_.param("base_frame",camera_frame_,std::string("/body"));
            nh_.param("gain", gain, 1.0);
            nh_.param("threshold_z", threshold_z, 0.1);
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&ArmHeightControl::pc_callback,this);
            twist_pub_ = nh_.advertise<geometry_msgs::Twist>("height_twist",1);

        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"arm_height_control");
    ArmHeightControl fp;

    ros::spin();
    return 0;
}
