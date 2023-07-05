#include "ros/ros.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include "tf/transform_broadcaster.h"


#define _freq 100


using namespace std;

class ODOMHEN {
	public:
		ODOMHEN();
		void cb(geometry_msgs::Twist::ConstPtr msg);
		void odom();
	private:
		float _x;
		float _y;
		float _theta;
		float _headingVel;
		float _turningVel;
		bool _topicActive;
		float _rescaleHeading;
		float _rescaleTurning;
		tf::Transform _trans;
		tf::TransformBroadcaster _trans_br;
		ros::NodeHandle _nh;
		ros::Subscriber _topic_sub;
		ros::Rate _rate;
};

ODOMHEN::ODOMHEN(): _rate(_freq) {
	_x = 0.04025;
	_y = 0;
	_theta = 0;
	_headingVel = 0;
	_turningVel = 0;
	_topicActive = false;
	_topic_sub = _nh.subscribe("/cmd_vel", 1, &ODOMHEN::cb, this);
	if (!_nh.getParam("rescaleHeading", _rescaleHeading)) {
		_rescaleHeading = 1;
	}
	if (!_nh.getParam("rescaleTurning", _rescaleTurning)) {
		_rescaleTurning = 1;
	}
	boost::thread(&ODOMHEN::odom, this);
}

//Callback function: the input of the function is the data to read
void ODOMHEN::cb(geometry_msgs::Twist::ConstPtr msg) {
	_turningVel = msg->angular.z*_rescaleTurning;
	_headingVel = msg->linear.x*_rescaleHeading;
	ROS_INFO("I heard: headingVel = %f, turningVel = %f", _headingVel, _turningVel);
}

void ODOMHEN::odom() {

float T = 1.0/(float)_freq;

	while (ros::ok()) {

		_x = _x + _headingVel*T*cos(_theta + 0.5*_turningVel*T);
		_y = _y + _headingVel*T*sin(_theta + 0.5*_turningVel*T);
		_theta = _theta + _turningVel*T;

		_trans.setOrigin(tf::Vector3(_x, _y, 0));
		tf::Quaternion q;
		q.setRPY(0, 0, _theta);
		_trans.setRotation(q);

		_trans_br.sendTransform(tf::StampedTransform(_trans,ros::Time::now(), "odomhen", "center_link"));

		_rate.sleep();
	}

}

int main( int argc, char** argv ) {

	//Init the ros node with ros_subscriber name
	ros::init(argc, argv, "Odomhen");

	//Create the ROS_SUB class object
	ODOMHEN odomhen;

	ros::spin();

	return 0;
}
