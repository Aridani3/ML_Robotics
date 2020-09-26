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
#include <time.h>

#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(trust_region_strategy, "levenberg_marquardt",
              "Options are: levenberg_marquardt, dogleg.");
DEFINE_string(dogleg, "traditional_dogleg", "Options are: traditional_dogleg,"
              "subspace_dogleg.");

DEFINE_bool(inner_iterations, false, "Use inner iterations to non-linearly "
            "refine each successful trust region step.");

DEFINE_string(blocks_for_inner_iterations, "automatic", "Options are: "
            "automatic, cameras, points, cameras,points, points,cameras");

DEFINE_string(linear_solver, "sparse_normal_cholesky", "Options are: "
              "sparse_schur, dense_schur, iterative_schur, sparse_normal_cholesky, "
              "dense_qr, dense_normal_cholesky and cgnr.");

DEFINE_string(preconditioner, "jacobi", "Options are: "
              "identity, jacobi, schur_jacobi, cluster_jacobi, "
              "cluster_tridiagonal.");

DEFINE_string(sparse_linear_algebra_library, "suite_sparse",
              "Options are: suite_sparse and cx_sparse.");

DEFINE_string(ordering, "automatic", "Options are: automatic, user.");

DEFINE_bool(robustify, false, "Use a robust loss function.");
DEFINE_double(eta, 1e-2, "Default value for eta. Eta determines the "
             "accuracy of each linear solve of the truncated newton step. "
             "Changing this parameter can affect solve performance.");

DEFINE_int32(num_threads, 1, "Number of threads.");
DEFINE_int32(num_iterations, 50, "Number of iterations.");
DEFINE_double(max_solver_time, 1e32, "Maximum solve time in seconds.");
DEFINE_bool(nonmonotonic_steps, false, "Trust region algorithm can use"
            " nonmonotic steps.");

struct PlaneError {
    // Create an error measurement to evalute point (x,y,z)
    PlaneError(double x, double y, double z, double weight=1)
        : x(x), y(y), z(z), weight(weight) { }

    template <typename T>

        bool operator()(const T* const w, T* residuals) const {

            residuals[0] =  T(z) - ( w[0]*T(x) + w[1]*T(y) + w[2] );

            return true;
        }

    double x,y,z, weight;
};

class FloorPlaneRegression {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;

        pcl::PointCloud<pcl::PointXYZ> lastpc_;
        ceres::Solver::Options options;

    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            //timer the get time of excution
            ros::WallTime start_, end_;

            start_ = ros::WallTime::now();

            // Receive the point cloud and convert it to the right format
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            // Linear regression:  z = a*x + b*y + c
            ceres::Problem problem;
            size_t n = lastpc_.size();
            // The parameter of the optimisation will be stored in the
            // following buffer:
            double X[3] = {0,0,0};
            for (unsigned int i=0;i<n;i++) {
                const pcl::PointXYZ & T = temp[i];
                double d = hypot(T.x,T.y);
                // In the sensor frame, this point would be inside the camera
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }
                // Measure the point distance in the base frame
                const pcl::PointXYZ & P = lastpc_[i];
                d = hypot(P.x,P.y);
                if (d>max_range_) {
                    // too far, ignore
                    continue;
                }
                ceres::LossFunction* loss_function;
                ceres::CostFunction *cost_function;
                loss_function = FLAGS_robustify ? new ceres::HuberLoss(1.0) : NULL;
                // TODO START
                // Use the PlaneError defined above to build an error term for
                // the ceres optimiser (see documentation link above)


                double x = P.x; double y = P.y; double z = P.z;
                cost_function = new ceres::AutoDiffCostFunction<PlaneError, 1, 3>(new PlaneError(x, y, z));


                // END OF TODO
                // This cost function is then added to the optimisation
                // problem, with X as a parameter
                problem.AddResidualBlock(cost_function,loss_function,X);
            }

            // Now we've prepared ceres' solver, we can just run it:
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // Assuming the result is computed in vector X
            ROS_INFO("Extracted floor plane: z = %.2fx + %.2fy + %.2f",
                    X[0],X[1],X[2]);

            // Same code as the linear regression: build the 3D orientation
            Eigen::Vector3f O,u,v,w;
            w << X[0],X[1],-1;
            w /= w.norm();
            // Assuming 
            O << 1.0, 0.0, 1.0*X[0]+0.0*X[1]+X[2];
            u << 2.0, 0.0, 2.0*X[0]+0.0*X[1]+X[2];
            u -= O;
            u /= u.norm();
            v = w.cross(u);

            tf::Matrix3x3 R(u(0),v(0),w(0),
                    u(1),v(1),w(1),
                    u(2),v(2),w(2));
            tf::Quaternion Q;
            R.getRotation(Q);
            
            // Now build a visualization marker
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
            m.color.g = 1.0;
            m.color.b = 0.0;

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
            // TODO: update these variable in the launch file, similarly to the
            // linear regression case
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            // Prepare the CERES options based on the command line arguments
            CHECK(StringToLinearSolverType(FLAGS_linear_solver,
                        &options.linear_solver_type));
            CHECK(StringToPreconditionerType(FLAGS_preconditioner,
                        &options.preconditioner_type));
            // CHECK(StringToSparseLinearAlgebraLibraryType(
            //             FLAGS_sparse_linear_algebra_library,
            //             &options.sparse_linear_algebra_library));
            // options.num_linear_solver_threads = FLAGS_num_threads;
            options.max_num_iterations = FLAGS_num_iterations;
            options.minimizer_progress_to_stdout = true;
            options.num_threads = FLAGS_num_threads;
            options.eta = FLAGS_eta;
            options.function_tolerance = 3e-4;
            options.max_solver_time_in_seconds = FLAGS_max_solver_time;
            options.use_nonmonotonic_steps = FLAGS_nonmonotonic_steps;
            options.update_state_every_iteration = true; 

            CHECK(StringToTrustRegionStrategyType(FLAGS_trust_region_strategy,
                        &options.trust_region_strategy_type));
            CHECK(StringToDoglegType(FLAGS_dogleg, &options.dogleg_type));
            options.use_inner_iterations = FLAGS_inner_iterations;

            // Now we can start the subscriber
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


