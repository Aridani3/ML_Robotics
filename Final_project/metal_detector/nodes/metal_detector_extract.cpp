#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>

#include <Eigen/Core>
#include <Eigen/Cholesky>


class MetalDetectorExtract {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber detector_sub_;

        ros::Subscriber info_sub_;
        ros::Subscriber scan_sub_;
        tf::TransformListener listener_;

        std::string sensor_frame_;
        std::string treasure_frame_;
        std::string outdir_;
        double min_displacement_;
        double min_measurement_;

        bool has_info;
        geometry_msgs::Pose2D last_pose;

    protected: // ROS Callbacks

        void md_callback(const std_msgs::Float32::ConstPtr& msg) {
            float metal_detector = msg->data;
            //ROS_INFO("metal_detector: %f", metal_detector);
            tf::StampedTransform tfw;
            try{
                listener_.waitForTransform(treasure_frame_,sensor_frame_,ros::Time(0),ros::Duration(1.0));
                listener_.lookupTransform(treasure_frame_,sensor_frame_,ros::Time(0),tfw);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }
            double d = hypot(tfw.getOrigin().x(), tfw.getOrigin().y());
            ROS_INFO("Distance to treasure: %f", d);
            // Check if we moved
            if (hypot(last_pose.x-tfw.getOrigin().x(), last_pose.y-tfw.getOrigin().y()) < min_displacement_) {
                return;
            }
            last_pose.x=tfw.getOrigin().x();
            last_pose.y=tfw.getOrigin().y();

            if (metal_detector >= min_measurement_){
                // Old fashion formatting
                char labelname[1024];
                sprintf(labelname,"%s/metal_detector_data.txt",outdir_.c_str());
                FILE * fp = fopen(labelname,"a");
                fprintf(fp,"%.3f %.3f\n",d, metal_detector);
                fclose(fp);
            }
        }


    public:
        MetalDetectorExtract() : nh_("~"){
            has_info = false;
            nh_.param("treasure_frame",treasure_frame_,std::string("/treasure_frame"));
            nh_.param("sensor_frame",sensor_frame_,std::string("/sensor_frame"));
            nh_.param("min_displacement",min_displacement_,0.01);
            nh_.param("min_measurement",min_measurement_,0.2);
            nh_.param("out_dir",outdir_,std::string("."));
            ros::Duration(0.5).sleep();

            // Reset label file
            char labelname[1024];
            sprintf(labelname,"%s/metal_detector_data.txt",outdir_.c_str());
            FILE * fp = fopen(labelname,"w");
            assert(fp);
            fclose (fp);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();
            tf::StampedTransform tfw;
            listener_.lookupTransform(treasure_frame_,sensor_frame_,ros::Time(0),tfw);
            last_pose.x=tfw.getOrigin().x()-1.0;
            last_pose.y=tfw.getOrigin().y()-1.0;

            detector_sub_ = nh_.subscribe("scans",1,&MetalDetectorExtract::md_callback,this);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"metal_detector_Extract");
    MetalDetectorExtract md;

    ros::spin();
    return 0;
}


