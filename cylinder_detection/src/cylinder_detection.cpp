#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>


class FloorPlaneRansac {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        tf::TransformListener listener_;
        tf::TransformListener listener_robot_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        std::string robot_frame_;
        double max_range_;
        double tolerance;
        int n_samples;

        pcl::PointCloud<pcl::PointXYZ> lastpc_;
        pcl::PointCloud<pcl::PointXYZ> robotpc_;

        std::vector<double> cylinders_a;
        std::vector<double> cylinders_b;
        std::vector<double> cylinders_r;

    protected: // ROS Callbacks

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
            
            // Find the floor

            n = pidx.size();
            size_t best = 0;
            size_t best2 = 0;
            double X[3] = {0,0,0};
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());

            unsigned int n_not_floor = 0;
            std::vector<size_t> pidx_not_floor;
                pidx_not_floor.clear();
                for (unsigned int i=0;i<(unsigned)n_samples;i++) {
                    size_t j = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
                    size_t k = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
                    size_t l = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);

                    while (j==k || k==l || j==l){
                        j = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
                        k = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
                        l = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
                    }

                    Eigen::Vector3f P1; P1 << lastpc_[pidx[j]].x, lastpc_[pidx[j]].y, lastpc_[pidx[j]].z;
                    Eigen::Vector3f P2; P2 << lastpc_[pidx[k]].x, lastpc_[pidx[k]].y, lastpc_[pidx[k]].z;
                    Eigen::Vector3f P3; P3 << lastpc_[pidx[l]].x, lastpc_[pidx[l]].y, lastpc_[pidx[l]].z;

                    Eigen::Vector3f U; U = P2-P1;
                    Eigen::Vector3f V; V = P3-P1;
                    Eigen::Vector3f N; N = U.cross(V);

                    float a = N(0);
                    float b = N(1);
                    float c = N(2);
                    float d = a*P1(0)+b*P1(1)+c*P1(2);

                    size_t counter = 0;

                    for (unsigned int m=0;m<(unsigned)n;m++) {
                        Eigen::Vector3f M; M << lastpc_[pidx[m]].x, lastpc_[pidx[m]].y, lastpc_[pidx[m]].z;
                        float distance = (M-P1).dot(N)/N.norm();
                        if (abs(distance) < tolerance) {
                            counter += 1;
                        }
                    }
                    if (counter > best) {
                        best = counter;
                        X[0] = -a/c;
                        X[1] = -b/c;
                        X[2] = d/c;
                    }
                        
                }
                // Exctact floor points
                for (unsigned int m=0;m<(unsigned)n;m++) {
                    Eigen::Vector3f M; M << lastpc_[pidx[m]].x, lastpc_[pidx[m]].y, lastpc_[pidx[m]].z;
                    if (abs (M[2] - (X[0]*M[0]+X[1]*M[1]+X[2])) > tolerance) {
                        pidx_not_floor.push_back(pidx[m]);
                        //ROS_INFO("Points: %.5f, %.5f, %.5f", M[0],M[1],M[2]);
                    }
                }
                n_not_floor = pidx_not_floor.size();
                ROS_INFO("Number of points outside the floor %d",(int)n_not_floor);

            // Find the cylinder
            best = 0;
            best2 = 0;
            double X_cylinder[3] = {0,0,0};

            if (n_not_floor > 10) {
                for (unsigned int i=0;i<(unsigned)n_samples;i++) {
                    // Pick 3 random points and check they are not the same
                    size_t j = std::min((rand() / (double)RAND_MAX) * n_not_floor,(double)n_not_floor-1);
                    size_t k = std::min((rand() / (double)RAND_MAX) * n_not_floor,(double)n_not_floor-1);
                    size_t l = std::min((rand() / (double)RAND_MAX) * n_not_floor,(double)n_not_floor-1);

                    while (j==k || k==l || j==l){
                        j = std::min((rand() / (double)RAND_MAX) * n_not_floor,(double)n_not_floor-1);
                        k = std::min((rand() / (double)RAND_MAX) * n_not_floor,(double)n_not_floor-1);
                        l = std::min((rand() / (double)RAND_MAX) * n_not_floor,(double)n_not_floor-1);
                    }

                    // Defining P1, P2, P3 such that P1, P2 are the farthest 
                    // and P1 is closer from the origine than P2
                    Eigen::Vector3f P;
                    Eigen::Vector3f P1; P1 << lastpc_[pidx_not_floor[j]].x, lastpc_[pidx_not_floor[j]].y, lastpc_[pidx_not_floor[j]].z;
                    Eigen::Vector3f P2; P2 << lastpc_[pidx_not_floor[k]].x, lastpc_[pidx_not_floor[k]].y, lastpc_[pidx_not_floor[k]].z;
                    Eigen::Vector3f P3; P3 << lastpc_[pidx_not_floor[l]].x, lastpc_[pidx_not_floor[l]].y, lastpc_[pidx_not_floor[l]].z;

                    float p12 = (P1 - P2).norm();
                    float p13 = (P1 - P3).norm();
                    float p23 = (P2 - P3).norm();
                    if ((p13 >= p12) && (p13 >= p23)){
                        P = P3;
                        P3 = P2;
                        P2 = P;
                    }
                    if ((p23 >= p12) && (p23 >= p13)){
                        P = P3;
                        P3 = P1;
                        P1 = P;
                    }
                    if (P2.norm() < P1.norm()){
                        P = P1;
                        P1 = P2;
                        P2 = P;
                    }

                    // Finding the center and radius of the circle
                    Eigen::Vector2f U; U << P2(0)-P1(0), P2(1)-P1(1);
                    float theta = atan(U(1)/U(0));
                    float xp3 = (P3(0)-P1(0))*cos(theta) + (P3(1)-P1(1))*sin(theta);
                    float yp3 = -(P3(0)-P1(0))*sin(theta) + (P3(1)-P1(1))*cos(theta);
                    float x1 = U.norm()/2;
                    float y1 = 0.5*(pow(xp3,2) - U.norm()*xp3 + pow(yp3,2)) / yp3;
                    float r = sqrt(pow(x1,2)+pow(y1,2));
                    float a = x1*cos(theta) - y1*sin(theta) + P1(0);
                    float b = x1*sin(theta) + y1*cos(theta) + P1(1);

                    size_t counter = 0;
                    size_t counter2 = 0;

                    for (unsigned int m=0;m<(unsigned)n_not_floor;m++) {
                        Eigen::Vector3f M; M << lastpc_[pidx_not_floor[m]].x, lastpc_[pidx_not_floor[m]].y, lastpc_[pidx_not_floor[m]].z;
                        float distance = abs(pow(r,2) - pow((M(0)-a),2) - pow((M(1)-b),2));
                        if (distance < tolerance) {
                            counter += 1;
                        }
                        if (distance < 5*tolerance) {
                            counter2 += 1;
                        }
                    }
                    if ((counter > 0.4*n_not_floor) && (counter >= 0.99*counter2) && (r > 0.05) && (r < 0.15)) {// && (X_cylinder[2] > 0.05) && (X_cylinder[2] < 0.15)) {
                        best = counter;
                        X_cylinder[0] = a;
                        X_cylinder[1] = b;
                        X_cylinder[2] = r;
                    }
                    if (counter > best2) {
                        best2 = counter2;
                    }
                    
                }
            }
            ROS_INFO("Best score %d", (int)best2);
            if (best > 0) {
                ROS_INFO("Cylinder detected");
                ROS_INFO("Equation of the cylinder: %.2f^2 = (x-%.2f)^2 + (y-%.2f)^2", X_cylinder[2], X_cylinder[0], X_cylinder[1]);
                bool add = true;
                for (unsigned int m=0;m<(unsigned)cylinders_a.size();m++) {
                    if (pow(cylinders_a[m]-X_cylinder[0],2) + pow(cylinders_b[m] - X_cylinder[1], 2)+ pow(cylinders_r[m] - X_cylinder[2], 2) < 0.5) {
                        add = false;
                    }
                }
                if (add == true) {
                    cylinders_a.push_back(X_cylinder[0]);
                    cylinders_b.push_back(X_cylinder[1]);
                    cylinders_r.push_back(X_cylinder[2]);
                }

            } else {
                ROS_INFO("No cylinder detected");
            }
            // End cylinder detection

            // Cylinder detected
            ROS_INFO("Cylinders detected:");
            for (unsigned int mm=0;mm<(unsigned)cylinders_a.size();mm++) {
                ROS_INFO("%.2f, %.2f, %.2f", cylinders_a[mm], cylinders_b[mm], cylinders_r[mm]);
            // End cylinder detected

            Eigen::Vector3f O,u,v,w;
            w << 0.0, 0.0, 1.0; //X[0], X[1], -1.0;
            w /= w.norm();
            O << cylinders_a[mm], cylinders_b[mm], 0.0;//X_cylinder[0], X_cylinder[1], 0.0; //1.0, 0.0, 1.0*X[0]+0.0*X[1]+X[2];
            u << 1.0, 0.0, 0.0; //2.0, 0.0, 2.0*X[0]+0.0*X[1]+X[2];
            //u -= O;
            u /= u.norm();
            v = w.cross(u);

            tf::Matrix3x3 R(u(0),v(0),w(0),
                    u(1),v(1),w(1),
                    u(2),v(2),w(2));
            tf::Quaternion Q;
            R.getRotation(Q);
            
            visualization_msgs::Marker m;
            m.header.stamp = msg->header.stamp;
            m.header.frame_id = base_frame_;
            m.ns = "cylinder_detector";
            m.id = mm;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = O(0);
            m.pose.position.y = O(1);
            m.pose.position.z = 0.0;//O(2);
            tf::quaternionTFToMsg(Q,m.pose.orientation);
            m.scale.x = cylinders_r[mm];//X_cylinder[2]; //1.0;
            m.scale.y = cylinders_r[mm];//X_cylinder[2]; //1.0;
            m.scale.z = 2.0;
            m.color.a = 0.5;
            m.color.r = 1.0;
            m.color.g = 1.0;
            m.color.b = 0.0;

            marker_pub_.publish(m);
            }
        }

    public:
        FloorPlaneRansac() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/world"));
            nh_.param("robot_frame",robot_frame_,std::string("/bubbleRob"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("n_samples",n_samples,1000);
            nh_.param("tolerance",tolerance,1.0);

            ROS_INFO("Searching for Plane parameter z = a x + b y + c");
            ROS_INFO("RANSAC: %d iteration with %f tolerance",n_samples,tolerance);
            assert(n_samples > 0);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneRansac::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("floor_plane",1);

        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_Ransac");
    FloorPlaneRansac fp;

    ros::spin();
    return 0;
}


