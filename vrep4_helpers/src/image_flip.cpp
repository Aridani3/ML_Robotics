#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

image_transport::Publisher *gpub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat output;
    cv::flip(cv_bridge::toCvShare(msg, msg->encoding)->image,
            output,0);
    sensor_msgs::ImagePtr I = cv_bridge::CvImage(msg->header, msg->encoding, output).toImageMsg();
    I->is_bigendian = msg->is_bigendian;
    gpub->publish(I);
      
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_flipper");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);
  image_transport::Publisher pub = it.advertise("image_flipped", 1);
  gpub = &pub;
  ros::spin();
}
