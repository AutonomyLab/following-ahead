#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"

class FisheyeToMono8
{
public:
  FisheyeToMono8(ros::NodeHandle nh) 
    : nh_(nh),
      image_transport_(nh_)
  {
    fisheye_subscriber = image_transport_.subscribe(
      "/camera/fisheye/image_raw", 1, 
      &FisheyeToMono8::imageCallback, this
    );
    pub_mono8_image_ = image_transport_.advertise("/camera/fisheye/mono8", 1);

  }

	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    // for 8UC1 type image (fisheye)
    if (msg->encoding == "8UC1")
    {
      sensor_msgs::ImagePtr mono_msg;
    
      cv::Mat mono_image;
      try
      {
        cv_bridge::CvImage cv_bridge_image;

        mono_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1)->image.clone();
        
        cv_bridge_image.image = mono_image;
        cv_bridge_image.encoding = "mono8";
        cv_bridge_image.header = msg->header;

        mono_msg = cv_bridge_image.toImageMsg();
      }
      catch (cv::Exception &e)
      {
        ROS_ERROR_STREAM("E: " << e.what());
      }

      pub_mono8_image_.publish(mono_msg);
    }
  }

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher pub_mono8_image_;
  image_transport::Subscriber fisheye_subscriber;
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "fisheye_to_mono8_publisher"); 
  ros::NodeHandle n("~");

  FisheyeToMono8 fisheye_to_mono8(n);

  ros::spin();
  return 0;
}