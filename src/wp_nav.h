#include "ros/ros.h"
#include "boost/thread.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "boost/thread.hpp"
//---mavros_msgs
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandTOL.h"
#include <mavros_msgs/PositionTarget.h>
//---
#include "utils.h"
#include "Eigen/Dense"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
//---


#include <mutex>          // std::mutex
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/Imu.h"
#include <ros/package.h>
#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;

class WP_NAV {
	public:
		WP_NAV();
		void position_controller();
		void run();
		void localization_cb ( geometry_msgs::PoseStampedConstPtr msg );
		void mavros_state_cb( mavros_msgs::State mstate);
		void takeoff( const double altitude );
		bool linear_motion( Vector3d dest, double yaw, double t_lock, bool heading );
		bool linear_motion( Vector3d dest, double t_lock );
		bool rotate( double angle, double t_lock );
		bool lock_rotation( double cmd, double t_lock);	
		bool yaw_reached (  double cmd, double mes  );	
		void select_action();
		void state_machine();
		void state_machine_2();
		void land();


	
	private:
		ros::NodeHandle _nh;
		ros::Publisher _target_pub;
		ros::Subscriber _localization_sub;
		ros::Subscriber _mavros_state_sub;
    bool _first_local_pos;

		// --- Desired state
		Vector3d _cmd_p;
		Vector3d _ref_p;
		Vector3d _ref_dp;
		double _cmd_yaw;
		int _rate;
		double _cruise_vel;
		// --- Drone state ---
		Vector3d _e_p;
		Vector3d _w_p;
		Vector4d _w_q;
		float _mes_yaw;
		Vector3d _w_lin_vel;
		Vector3d _w_ang_vel;
		double _yaw_motion_threshold;
		double _linear_motion_threshold;
    mavros_msgs::State _mstate;

		ros::ServiceClient _arming_client;
		ros::ServiceClient _set_mode_client;
		ros::ServiceClient _land_client;
};




