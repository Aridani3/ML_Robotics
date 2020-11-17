#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>


struct comp 
{
	template<typename T>
	bool operator()(const T &lhs, const T &rhs) const
	{
		if (lhs.x == rhs.x)
			return lhs.y > rhs.y;

		return lhs.x < rhs.x;
	}
};

class OccGridMapping {
    protected:
        ros::Subscriber scan_sub_;
        tf::TransformListener listener_;
        tf::TransformListener listener_robot_;

        ros::NodeHandle nh_;
        
        ros::Publisher image_pub_;
        std::string base_frame_;
        std::string robot_frame_;
        double max_range_;

        pcl::PointCloud<pcl::PointXYZ> lastpc_;
        pcl::PointCloud<pcl::PointXYZ> robotpc_;

        typedef std::vector<pcl::PointXYZ> PointList;
        typedef std::map<cv::Point, PointList, comp>  ListMatrix;
        ListMatrix M;
        cv::Mat_<uint8_t> traversability_map;
        cv::Mat_<int32_t> lprobability_map;

        double grid_size;
        int beta, gama;

    protected: // ROS Callbacks

        float traversable(float angle){
            return exp(-100 * (angle - M_PI/9)) / ( 1 + exp(-100 * (angle - M_PI/9)) );
        }

        float lprobability(float angle, float d){
            float trav = traversable(angle);
            float proba = trav * (0.9 - 0.4 * exp(-gama/d)) + (1-trav) * (0.1 + 0.4 * exp(-gama/d)); 
            return log(proba / (1 - proba));
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            listener_robot_.waitForTransform(robot_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(robot_frame_,msg->header.stamp, temp, msg->header.frame_id, robotpc_, listener_robot_);

            //
            unsigned int n = temp.size();
            std::vector<size_t> pidx;
            // First count the useful points
            for (unsigned int i=0;i<n;i++) {
                float x = temp[i].x;
                float y = temp[i].y;
                float d = hypot(x,y);
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }

                x = robotpc_[i].x;
                y = robotpc_[i].y;
                d = hypot(x,y);
                if (d > max_range_) {
                    // too far, ignore
                    continue;
                }
                pidx.push_back(i);
            }

            n = pidx.size(); 
            //ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());

            M.clear();

            for (unsigned int i=0;i<n;i++) {
                float x = lastpc_[pidx[i]].x;
                float y = lastpc_[pidx[i]].y;
                float epsilon(1e-2);
                if ( ( (x-5) < -epsilon && (x+5) > epsilon) && ( (y-5)<-epsilon && (y + 5)>epsilon) ) {
                    cv::Point coord((x + 5)/grid_size, (y + 5)/grid_size);

                    if ( M[coord].empty()){
                        M[coord].push_back(robotpc_[pidx[i]]); // to get distance between robot and grid in the lists head
                        M[coord].push_back(lastpc_[pidx[i]]);
                    }else{    
                        M[coord].push_back(lastpc_[pidx[i]]);
                    }
                }
            }
            


            for (ListMatrix::const_iterator it=M.begin(); it!=M.end(); it++) {    
                cv::Point coord = it->first;
                const PointList L = it->second;
                
                int k = L.size();
                    
                pcl::PointXYZ robot_point = L.front();
                float d = hypot(robot_point.x, robot_point.y);

                if (k > 5){

                    float min_point = L.back().z;
                    float max_point = L.back().z;

                    for (PointList::const_iterator jt=L.begin()+1;jt!=L.end();jt++) {  

                        double z = jt->z;       

                        if (z > max_point){ max_point = z; }
                        if (z < min_point){ min_point = z; }
                    }    

                    float angle = atan2(max_point - min_point, 0.1);
                    //ROS_INFO("lprobability %d", lprobability_map(coord.x, coord.y));
                    lprobability_map(coord.x, coord.y) = lprobability_map(coord.x, coord.y) + lprobability(angle, d)*127/2.5;
                    //ROS_INFO("lprobability: %f --> %d", lprobability(angle, d), lprobability_map(coord.x, coord.y));
                    traversability_map(coord.x, coord.y) = (1 - ( 1 / (1 + exp(lprobability_map(coord.x, coord.y)*2.5/127)))) * 255;
                    //traversability_map(coord.x, coord.y) = traversable(angle) * 255;
                }
            }
            
            sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "mono8", traversability_map).toImageMsg();
            image_pub_.publish(image);
            
        }

    public:
        OccGridMapping() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/world"));
            nh_.param("robot_frame",robot_frame_,std::string("/bubbleRob"));
            nh_.param("grid_size", grid_size, 0.1);
            nh_.param("max_range",max_range_,3.0);
            nh_.param("beta",beta,10); // constant used in traversibily function of an angle
            nh_.param("gama",gama,1); // constant used in log proba function

            int map_size = (int) (10 / grid_size);
            traversability_map = cv::Mat_<uint8_t>(map_size, map_size,128);
            //lprobability_map = cv::Mat::zeros(map_size, map_size, CV_8S);
            lprobability_map = cv::Mat(map_size, map_size, CV_32S, cv::Scalar(0));

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&OccGridMapping::pc_callback,this);

            image_pub_ = nh_.advertise<sensor_msgs::Image>("/occupancy_grid", 1);

        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"traversability_occgrid");
    OccGridMapping om;
    ros::spin();
    return 0;
}

