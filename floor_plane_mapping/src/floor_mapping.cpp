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


struct comp //https://www.techiedelight.com/use-custom-objects-keys-std-map-cpp/
{
	template<typename T>
	bool operator()(const T &lhs, const T &rhs) const
	{
		if (lhs.x == rhs.x)
			return lhs.y > rhs.y;

		return lhs.x < rhs.x;
	}
};

class FloorPlaneHough {
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

        cv::Mat_<uint32_t> accumulator;

        typedef std::vector<pcl::PointXYZ> PointList;
        typedef std::map<cv::Point, PointList, comp>  ListMatrix;
        ListMatrix M;
        cv::Mat_<uint8_t> traversability_map;
        
        int n_a, n_b, n_c;
        double a_min, a_max, b_min, b_max, c_min, c_max;

    protected: // ROS Callbacks

        float traversable(float angle){
            return exp(-100 * (angle - M_PI/6)) / ( 1 + exp(-100 * (angle - M_PI/6)) );
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
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());


            for (unsigned int i=0;i<n;i++) {
                float x = lastpc_[pidx[i]].x;
                float y = lastpc_[pidx[i]].y;

                float epsilon(1e-2);
                if ( ( (x-5) < -epsilon && (x+5) > epsilon) && ( (y-5)<-epsilon && (y + 5)>epsilon) ) {
                    cv::Point coord((x + 5)*10, (y + 5)*10);    
                    M[coord].push_back(lastpc_[pidx[i]]);
                }
            }
            ROS_INFO("Map populated");


            for (ListMatrix::const_iterator it=M.begin(); it!=M.end(); it++) {    
                cv::Point coord = it->first;
                const PointList L = it->second;
                
                int k = L.size();

                if (k > 10){
                    
                    pcl::PointXYZ min_point = L.front();
                    pcl::PointXYZ max_point = L.front();

                    for (PointList::const_iterator jt=L.begin();jt!=L.end();jt++) {  
                                
                        double z = jt->z;       

                        if (z > max_point.z){ max_point = *jt;}
                        if (z < min_point.z){ min_point = *jt;}


                    }    
                    float angle = atan2(max_point.z - min_point.z, 0.1);

                    //Eigen::Vector3f N; N << -X[0], -X[1], 1.0;
                    //Eigen::Vector3f G; G << 0, 0, 1;
                    //float angle = acos(1 / N.norm());
                    traversability_map(coord.x, coord.y) = traversable(angle) * 255;
                    //ROS_INFO("Grid (%d, %d, %d) angle %f  trav %f", coord.x, coord.y, (int)L.size(), angle*180/M_PI, traversable(angle));
                }
            }
            

            //cv::imshow("Traversability_map", traversability_map);
            
            sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "mono8", traversability_map).toImageMsg();
            image_pub_.publish(image);
            
        }

    public:
        FloorPlaneHough() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/world"));
            nh_.param("robot_frame",robot_frame_,std::string("/bubbleRob"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("n_a",n_a,10);
            nh_.param("a_min",a_min,-1.0);
            nh_.param("a_max",a_max,+1.0);
            nh_.param("n_c",n_c,10);
            nh_.param("c_min",c_min,-1.0);
            nh_.param("c_max",c_max,+1.0);

            assert(n_a > 0);
            assert(n_b > 0);
            assert(n_c > 0);

            ROS_INFO("Searching for Plane parameter z = a x + b y + c");
            ROS_INFO("a: %d value in [%f, %f]",n_a,a_min,a_max);
            ROS_INFO("b: %d value in [%f, %f]",n_b,b_min,b_max);
            ROS_INFO("c: %d value in [%f, %f]",n_c,c_min,c_max);

            // the accumulator is created here as a 3D matrix of size n_a x n_b x n_c
            int dims[3] = {n_a,n_b,n_c};
            accumulator = cv::Mat_<uint32_t>(3,dims);
            traversability_map = cv::Mat_<uint8_t>(100,100,128);



            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneHough::pc_callback,this);

            image_pub_ = nh_.advertise<sensor_msgs::Image>("/traversability_map", 1);

        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_mapping");
    FloorPlaneHough fp;
    //cv::namedWindow( "Traversability_map", CV_WINDOW_AUTOSIZE );
    ros::spin();
    return 0;
}

