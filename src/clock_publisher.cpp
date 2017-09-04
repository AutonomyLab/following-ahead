#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <boost/asio.hpp>

// milliseconds
#define INTERVAL 20.0

boost::asio::io_service io_service;
boost::posix_time::milliseconds interval(INTERVAL); //  millisecond
boost::asio::deadline_timer timer(io_service, interval);

double time_elapsed = 0;
ros::Publisher clock_pub;

void tick(const boost::system::error_code& /*e*/) 
{
    time_elapsed += INTERVAL;

    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = ros::Time(time_elapsed/1e3);
    clock_pub.publish(clock_msg);


    // Reschedule the timer for 1 second in the future:
    timer.expires_at(timer.expires_at() + interval);
    // Posts the timer event
    timer.async_wait(tick);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "clock_publisher"); 
  	ros::NodeHandle n("~");

  	clock_pub = n.advertise<rosgraph_msgs::Clock>("/clock", 10);

  	timer.async_wait(tick);

  	io_service.run();
    return 0;
}