#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Dense> 
#include <time.h>

class FloorPlaneRegression {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;

        pcl::PointCloud<pcl::PointXYZ> lastpc_;

    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            //timer
            ros::WallTime start_, end_;

            start_ = ros::WallTime::now();
            
            // Receive the point cloud and convert it to the right format
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            //
            unsigned int n = temp.size();
            std::vector<size_t> pidx;
            // First count the useful points
            for (unsigned int i=0;i<n;i++) {
                float x = temp[i].x;
                float y = temp[i].y;
                float d = hypot(x,y);
                // In the sensor frame, this point would be inside the camera
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }
                // Measure the point distance in the base frame
                x = lastpc_[i].x;
                y = lastpc_[i].y;
                d = hypot(x,y);
                if (d > max_range_) {
                    // too far, ignore
                    continue;
                }
                // If we reach this stage, we have an acceptable point, so
                // let's store it
                pidx.push_back(i);
            }
            
            //
            //
            // TODO START
            // 
            // Linear regression: z = a*x + b*y + c
            // Update the code below to use Eigen to find the parameters of the
            // linear regression above. 
            //
            // n is the number of useful point in the point cloud
            n = pidx.size();
            // Eigen is a matrix library. The line below create a 3x3 matrix A,
            // and a 3x1 vector B
            Eigen::MatrixXf A(n,3);
            Eigen::MatrixXf B(n,1);
            for (unsigned int i=0;i<n;i++) {
                // Assign x,y,z to the coordinates of the point we are
                // considering.
                double x = lastpc_[pidx[i]].x;
                double y = lastpc_[pidx[i]].y;
                double z = lastpc_[pidx[i]].z;

                // Example of initialisation of the matrices (wrong)
                A(i,0) = x;
                A(i,1) = y;
                A(i,2) = 1;

                B(i,0) = z;
            }
            Eigen::MatrixXf X = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
            //Eigen::MatrixXf X = (A.transpose() * A).inverse() * A.transpose() * B
            
            // Details on linear solver can be found on 
            // http://eigen.tuxfamily.org/dox-devel/group__TutorialLinearAlgebra.html
            
            // Assuming the result is computed in vector X
            ROS_INFO("Extracted floor plane: z = %.2fx + %.2fy + %.2f",
                    X(0),X(1),X(2));

            // END OF TODO

            // Now build an orientation vector to display a marker in rviz
            // First we build a basis of the plane normal to its normal vector
            Eigen::Vector3f O,u,v,w;
            w << X(0), X(1), -1.0;
            w /= w.norm();
            O << 1.0, 0.0, 1.0*X(0)+0.0*X(1)+X(2);
            u << 2.0, 0.0, 2.0*X(0)+0.0*X(1)+X(2);
            u -= O;
            u /= u.norm();
            v = w.cross(u);
            // Then we build a rotation matrix out of it
            tf::Matrix3x3 R(u(0),v(0),w(0),
                    u(1),v(1),w(1),
                    u(2),v(2),w(2));
            // And convert it to a quaternion
            tf::Quaternion Q;
            R.getRotation(Q);
            
            // Documentation on visualization markers can be found on:
            // http://www.ros.org/wiki/rviz/DisplayTypes/Marker
            visualization_msgs::Marker m;
            m.header.stamp = msg->header.stamp;
            m.header.frame_id = base_frame_;
            m.ns = "floor_plane";
            m.id = 1;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = O(0);
            m.pose.position.y = O(1);
            m.pose.position.z = O(2);
            tf::quaternionTFToMsg(Q,m.pose.orientation);
            m.scale.x = 1.0;
            m.scale.y = 1.0;
            m.scale.z = 0.01;
            m.color.a = 0.5;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 1.0;

            //fin du timer 
            end_ = ros::WallTime::now();

            // print results
            double execution_time = (end_ - start_).toNSec() * 1e-6;
            ROS_INFO("Exectution time (ms): %.2f", execution_time); 

            // Finally publish the marker
            marker_pub_.publish(m);
            
        }

    public:
        FloorPlaneRegression() : nh_("~") {
            // TODO START
            // The parameter below described the frame in which the point cloud
            // must be projected to be estimated. You need to understand TF
            // enough to find the correct value to update in the launch file
            nh_.param("base_frame",base_frame_,std::string("/body"));
            // This parameter defines the maximum range at which we want to
            // consider points. Experiment with the value in the launch file to
            // find something relevant.
            nh_.param("max_range",max_range_,5.0);
            // END OF TODO

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            // Subscribe to the point cloud and prepare the marker publisher
            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneRegression::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("floor_plane",1);

        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_regression");
    FloorPlaneRegression fp;

    ros::spin();
    return 0;
}


