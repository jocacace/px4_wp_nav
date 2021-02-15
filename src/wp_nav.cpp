#include "wp_nav.h"



WP_NAV::WP_NAV() {

    if( !_nh.getParam("rate", _rate)) {
        _rate = 100;
    }


    if(!_nh.getParam("yaw_motion_threshold", _yaw_motion_threshold) ) {
        _yaw_motion_threshold = 0.25;
    }

    if(!_nh.getParam("linear_motion_threshold", _linear_motion_threshold)) {
        _linear_motion_threshold = 0.15;
    }

		_first_local_pos = false;
    _target_pub = _nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    _localization_sub = _nh.subscribe( "/mavros/vision_pose/pose", 1, &WP_NAV::localization_cb, this);
    _mavros_state_sub = _nh.subscribe( "/mavros/state", 1, &WP_NAV::mavros_state_cb, this);


		// --- Services ---
    _arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    _land_client = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    //---
}


void WP_NAV::mavros_state_cb( mavros_msgs::State mstate) {
    _mstate = mstate;
}


void WP_NAV::localization_cb ( geometry_msgs::PoseStampedConstPtr msg ) {

    _w_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    
    Eigen::Vector3d rpy = utilities::R2XYZ ( utilities::QuatToMat ( Eigen::Vector4d( msg->pose.orientation.w,  msg->pose.orientation.x,  msg->pose.orientation.y,  msg->pose.orientation.z) ) );
    _mes_yaw = rpy(2);

    Quaternionf q;
    q = AngleAxisf(0.0, Vector3f::UnitX())
        * AngleAxisf(0.0, Vector3f::UnitY())
        * AngleAxisf(_mes_yaw, Vector3f::UnitZ());
    Vector4d w_q ( q.w(), q.x(), q.y(), q.z() );
    _w_q = w_q / w_q.norm() ;

   
    _first_local_pos = true;

    
}

void WP_NAV::takeoff( const double altitude ) {
  
  ros::Rate rate(10);
  
  //Set up control mode
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  //---

  if( _set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
    ROS_INFO("OFFBOARD mode enabled");
  }



  //---Arm
  if( _arming_client.call(arm_cmd) && arm_cmd.response.success){
  }

  while(!_mstate.armed ) usleep(0.1*1e6);
  ROS_INFO("Vehicle armed");
  //---

  while(_mstate.mode != "OFFBOARD" ) usleep(0.1*1e6);
  ROS_INFO("Vehicle in offboard");

  _cmd_p(2) = altitude;
  
  while( fabs( _cmd_p(2) - _w_p(2)) > 0.1 ) {
    usleep(0.1*1e6);
  }

  ROS_INFO("Takeoff completed");
  
}

/* 
 * Wait the rotation is completed
 *  t_lock < 0 - no lock action
 *  t_lock = 0 - lock until rotation is completed 
 *  t_lock > 0 - wait until the elapsed time is completed
 */
bool WP_NAV::yaw_reached (  double cmd, double mes  )  {


  double c_mes_yaw = mes;
  
  if ( c_mes_yaw < 0.0 ) {
    c_mes_yaw += 2*M_PI;
  } 
  double dist = std::min(  fabs(  cmd - (mes-2*M_PI)   ), std::min( fabs(cmd - mes ), fabs( cmd - (mes+2*M_PI))  ) );
  return dist < _yaw_motion_threshold; 

}


bool WP_NAV::lock_rotation( double cmd, double t_lock) {
 

  if( t_lock >= 0.0) {
    ros::Rate r(10);

    if( t_lock == 0.0 ) {   
      while( !yaw_reached(_cmd_yaw, _mes_yaw ))
        r.sleep();
      return true;

    } //Motion completed without maximum elapsed time
    else {
      double t = 0.0;
      double Ts = 1.0/10.0;
      while( t < t_lock && !yaw_reached(_cmd_yaw, _mes_yaw ) ) {

        t += Ts;
        r.sleep();
      } //wait until t_lock time

      if( t > t_lock ) {
        ROS_WARN("Rotation not completed, it takes too much time");
        return false;
      } //rotation not completed yet, but return
    }
  }
  return true;
} //Wait until the motion rotation is completed: [no_wait, wait_forever, wait_until_t_lock]


bool WP_NAV::rotate( double angle, double t_lock ) {
  _cmd_yaw  = angle;
  return lock_rotation(_cmd_yaw, t_lock);
} //Rotate of a desired angle

bool WP_NAV::linear_motion( Vector3d dest, double yaw, double t_lock, bool heading ) {

 
    bool rot = false;

    if( heading ) {
				//---Rotation first: retrieve desired rotation angle
				Vector3d v;
				v = Vector3d( dest(0) - _w_p(0), dest(1) - _w_p(1), 0.0 );
				v = v / v.norm();
 				rot = rotate(atan2(v(1), v(0) ), 0.0 );
		}    
		else 
 				rot = rotate(yaw, 0.0 );

    if( rot ) {

        _cmd_p(0) = dest(0);
        _cmd_p(1) = dest(1);
        _cmd_p(2) = dest(2);

        ros::Rate r(10);

        if( t_lock < 0.0 ) {
            return true;
        }
        else if( t_lock > 0.0 ) {
            float t = 0.0;
            float Ts = 1.0/10.0;

            while( t<t_lock && ( dest - _w_p ).norm() > _linear_motion_threshold ) {
                t+=Ts;
                r.sleep();
            }
        }
        else {
            while( ( dest - _w_p ).norm()  > _linear_motion_threshold ) {
                r.sleep();
            }
        }
    }
    
    return true;
} //Linear motion: first rotation, than navigation


bool WP_NAV::linear_motion( Vector3d dest, double t_lock ) {

		_cmd_p(0) = dest(0);
		_cmd_p(1) = dest(1);
		_cmd_p(2) = dest(2);

		ros::Rate r(10);

		if( t_lock < 0.0 ) {
		    return true;
		}
		else if( t_lock > 0.0 ) {
		    float t = 0.0;
		    float Ts = 1.0/10.0;

		    while( t<t_lock && ( dest - _w_p ).norm() > _linear_motion_threshold ) {
		        t+=Ts;
		        r.sleep();
		    }
		}
		else {
		    while( ( dest - _w_p ).norm()  > _linear_motion_threshold ) {
		        r.sleep();
		    }
		}


	return true;
} //Linear motion: first rotation, than navigation


void WP_NAV::position_controller(){

    ros::Rate r(10);
    double ref_T = 1/10.0;

    double ref_omega0_xyz;
    if( !_nh.getParam("ref_omega0_xyz", ref_omega0_xyz)) {
        ref_omega0_xyz = 1.0;
    }
    double ref_zita;
    if( !_nh.getParam("ref_zita", ref_zita)) {
        ref_zita = 0.75;
    }
    double ref_jerk_max;
    if( !_nh.getParam("ref_jerk_max", ref_jerk_max)) {
        ref_jerk_max = 15.0;
    }
    double ref_acc_max;
    if( !_nh.getParam("ref_acc_max", ref_acc_max)) {
        ref_acc_max = 5.0;
    }
    double ref_vel_max;
    if( !_nh.getParam("ref_vel_max", ref_vel_max)) {
        ref_vel_max = 2.0;
    }
    if( !_nh.getParam("max_cruise_vel", _cruise_vel)) {
        _cruise_vel = 2.0;
    }

    mavros_msgs::PositionTarget ptarget;
    ptarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    ptarget.type_mask =
    mavros_msgs::PositionTarget::IGNORE_VX |
    mavros_msgs::PositionTarget::IGNORE_VY |
    mavros_msgs::PositionTarget::IGNORE_VZ |
    mavros_msgs::PositionTarget::IGNORE_AFX |
    mavros_msgs::PositionTarget::IGNORE_AFY |
    mavros_msgs::PositionTarget::IGNORE_AFZ |
    mavros_msgs::PositionTarget::FORCE |
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE; // |

    while( !_first_local_pos )
        usleep(0.1*1e6);
    ROS_INFO("First local pose arrived!");

    _cmd_p = _ref_p = _w_p;

    Vector3d ref_dp;
    Vector3d ref_ddp;
    _e_p = Vector3d(0.0, 0.0, 0.0);

    _cmd_yaw = _mes_yaw;

    while (ros::ok()) {


				//cout << "_mes_yaw: " << _mes_yaw << endl;
        if( _mstate.mode != "OFFBOARD" ) {
            _ref_p = _cmd_p = _w_p;
            _cmd_yaw = _mes_yaw;
            ref_dp = Vector3d(0.0, 0.0, 0.0);
            ref_ddp = Vector3d(0.0, 0.0, 0.0);
        }
        else {
            //---Position
            Vector3d ddp = Vector3d(0.0, 0.0, 0.0);
            Vector3d dp  = Vector3d(0.0, 0.0, 0.0);
            
            //Errore di posizione
            _e_p = _cmd_p - _ref_p;    
            ddp(0) = ref_omega0_xyz * ref_omega0_xyz * _e_p(0) - 2.0 * ref_zita * ref_omega0_xyz * ref_dp(0);
            ddp(1) = ref_omega0_xyz * ref_omega0_xyz * _e_p(1) - 2.0 * ref_zita * ref_omega0_xyz * ref_dp(1);
            ddp(2) = ref_omega0_xyz * ref_omega0_xyz * _e_p(2) - 2.0 * ref_zita * ref_omega0_xyz * ref_dp(2);
            
            Vector3d jerk = (ddp - ref_ddp)/ref_T;
            double n_jerk = jerk.norm();
    
            if( n_jerk > ref_jerk_max) {
                jerk *= (ref_jerk_max/n_jerk);
            }
            
            ddp = ref_ddp + jerk*ref_T;

            double sa = 1.0;
            double n_acc = ddp.norm();
            if(n_acc > ref_acc_max) {
                sa = ref_acc_max/n_acc ;
            }
            ref_ddp = ddp * sa;
                        
            dp = ref_dp + ref_ddp * ref_T;
            double n_vel = dp.norm();

            if(n_vel > _cruise_vel ) {
                for(int i = 0; i<3; i++) {
                    if(ref_dp[i] * _e_p[i] > 0) {
                        ref_ddp[i] = 0.0;
                    }
                }
                dp = ref_dp + ref_ddp*ref_T;
                ref_dp = dp / dp.norm() * _cruise_vel;
            }
            else {
                //Aggiornamento velocita'
                ref_dp += ref_ddp * ref_T ;
            }
            //Aggiornamento posizione
            _ref_p  += ref_dp*ref_T;
            //----
        }

        //---Publish command
        ptarget.header.stamp = ros::Time::now();
        ptarget.position.x = _ref_p[0];
        ptarget.position.y = _ref_p[1];
        ptarget.position.z = _ref_p[2];
        ptarget.yaw = _cmd_yaw;
        _target_pub.publish( ptarget );
        //---

        r.sleep();
    }
}



void WP_NAV::state_machine() {

	/*
	string line = "";
	cout << "Type s to start" << endl;

	while ( line != "s" ) {
		getline( cin, line );
	}
	*/
	takeoff(3.0);

	cout << "GOto first wp!" << endl;
	linear_motion( Vector3d( -0.39, 2.61, 3.0 ), 0.0);
	_cruise_vel = 0.1;
	cout << "GOto second wp!" << endl;
	linear_motion( Vector3d( -0.39, 2.61, 1.5 ), 0.0);
	/*
	Landing position: 
    x: -0.391391575336
    y: 2.61469936371
    z: 2.25313782692	
	*/
}



void WP_NAV::state_machine_2() {

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";


 if( _set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
    ROS_INFO("OFFBOARD mode enabled");
  }



  //---Arm
  if( _arming_client.call(arm_cmd) && arm_cmd.response.success){
  }

  while(!_mstate.armed ) usleep(0.1*1e6);
  ROS_INFO("Vehicle armed");
 
	_cruise_vel = 0.3;
	cout << "GOto first wp!" << endl;
	linear_motion( Vector3d( _cmd_p(0), _cmd_p(1), _cmd_p(2) + 1.0 ), 0.0);

	cout << "GOto second wp!" << endl;
	linear_motion( Vector3d( 0.1, 0.4, _cmd_p(2) ), 0.0);

	land();
	/*
	Landing position: 
    x: -0.391391575336
    y: 2.61469936371
    z: 2.25313782692	
	*/
}



void WP_NAV::select_action() {

  string line;

  while(ros::ok()) {
    
    cout << "--------------------" << endl;
    cout << "Insert new action: " << endl;
    cout << "1 - takeoff" << endl;
    cout << "2 - land" << endl;
    cout << "3 - rotate" << endl;
    cout << "4 - move" << endl;
		cout << "5 - move along positive x" << endl;
		cout << "6 - move along positive y" << endl;
		cout << "7 - move along positive z" << endl;
		cout << "8 - machine 1" << endl;
		cout << "9 - machine 2" << endl;
    cout << "--------------------" << endl;

    getline(cin, line);

    if( line == "1" ) {
        takeoff(1.8);
    }
    else if( line == "2") {
      land();
    }
    else if( line == "3" ) {
      cout << "Insert desired angle" << endl;
      getline(cin, line);
      rotate( stod(line), 0.0);
    }
    else if( line == "4" ) {

      cout << "Insert destionation point" << endl;
      float x, y, z;
      scanf("%f %f %f", &x, &y, &z );

      Vector3d dest;
      dest = Vector3d ( x, y, z );
      linear_motion ( dest, 0.0 );
    }

		else if( line == "5" ) {
			cout << "Move along positive x" << endl;
			linear_motion( Vector3d( _cmd_p(0) + 1.0, _cmd_p(1), _cmd_p(2)), 0.0 );
		}	
		else if( line == "6" ) {
			cout << "Move along positive y" << endl;
			linear_motion( Vector3d( _cmd_p(0), _cmd_p(1) + 1.0, _cmd_p(2)), 0.0 );
		}
		else if( line == "7" ) {
			cout << "Move along positive z" << endl;
			linear_motion( Vector3d( _cmd_p(0), _cmd_p(1), _cmd_p(2) + 1.0), 0.0 );
		}

		else if( line == "8") {
			boost::thread state_machine_t( &WP_NAV::state_machine, this );
		}
		else if( line == "9") {
			boost::thread state_machine_t2( &WP_NAV::state_machine_2, this );

		}
  }
}

void WP_NAV::land() {
  mavros_msgs::CommandTOL land_srv;
  _land_client.call( land_srv );  


  //wait for landing
  cout << "Waiting disarm" << endl;
  while( _mstate.armed ) usleep(0.1*1e6);
  cout << "Disarmed!" << endl;

}




void WP_NAV::run(){
    boost::thread position_controller_t( &WP_NAV::position_controller, this);
		//boost::thread state_machine_t( &WP_NAV::state_machine, this );
		boost::thread select_action_t( &WP_NAV::select_action, this );
    ros::spin();
}

int main(int argc, char** argv ) {


    ros::init(argc, argv, "exploration");
    WP_NAV wpnav;
    
    wpnav.run();

    return 0;

}
