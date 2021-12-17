#include <iostream>
#include <map>
#include <math.h>
#include <chrono>

#include "ros/ros.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
// #include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include "mpc_controller.h"
#include <Eigen/Core>
#include <Eigen/QR>

using namespace std;
using namespace Eigen;

/********************/
/* CLASS DEFINITION */
/********************/
class MPCNode
{
    public:
        MPCNode(ros::NodeHandle, ros::NodeHandle);
        int num_thread(){ return num_threads_;};
        
    private:
        ros::Subscriber odom_sub_,
                        path_planner_sub_, 
                        goal_sub_, 
                        _sub_amcl;
        ros::Publisher  mpc_ref_pub_, 
                        twist_cmd_pub_, 
                        mpc_traj_pub_, 
                        twist_stmp_pub_,
                        _pub_ackermann;
        
        ros::Timer _timer1;
        tf::TransformListener _tf_listener;

        geometry_msgs::PoseStamped    _goal_pos;
        nav_msgs::Odometry      _odom;                      // current loc.
        nav_msgs::Path          _odom_path,                 // waypoint path
                                _mpc_traj;
        geometry_msgs::Twist    _twist_msg;
        geometry_msgs::TwistStamped 
                                _twist_stmp_msg;

        string                  path_topic_,
                                odom_topic_, 
                                goal_topic_,
                                cmd_topic_;
        string                  map_frame_, 
                                odom_frame_, 
                                car_frame_;

        
        
        MPC mpc_solver_;


        map<string, double> _mpc_params;
        
        double          _mpc_steps, 
                        _ref_cte, 
                        _ref_etheta, 
                        _ref_vel, 
                        _w_cte, 
                        _w_etheta, 
                        _w_vel, 
                        _w_angvel, 
                        _w_accel, 
                        _w_angvel_d, 
                        _w_accel_d, 
                        _max_angvel, 
                        _max_throttle, 
                        _bound_value,
                        
                        inplace_rotate_vel_;

        //double _Lf; 
        double          _dt, 
                        _w, 
                        _throttle, 
                        _speed, 
                        _max_speed,
                        _pathLength, 
                        goal_tolerance_, 
                        _waypoint_dist_;

        int             _controller_freq, 
                        _downSampling, 
                        num_threads_,  // Number of threads for this MPC node
                        debug_level_;

        
        bool            _goal_received, 
                        _goal_reached, 
                        _path_computed, 
                        is_pub_twist_,
                        _delay_mode, 
                        _onetime_noti;

        // //For making global planner
        // nav_msgs::Path _gen_path;
        

        double          _polyeval(Eigen::VectorXd , double );
        Eigen::VectorXd _polyfit(Eigen::VectorXd , Eigen::VectorXd , int );


        // Callback func.
        void odomCB(const nav_msgs::Odometry::ConstPtr& );
        void pathCB(const nav_msgs::Path::ConstPtr& );
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& );

        double  _getYaw(const geometry_msgs::Pose&);
        void    _mpcCompute();
        void    _publishTwist(double, double);
        void    _printParams();

        void controlLoopCB(const ros::TimerEvent&);
        void makeGlobalPath(const nav_msgs::Odometry );

        inline double  _fix_angle(double angle){
            angle = (angle > M_PI)? -2*M_PI + angle: angle;
            angle = (angle < -M_PI)? 2*M_PI -angle: angle;
            return angle;
        }
        

}; // end of class


MPCNode::MPCNode(ros::NodeHandle nh, ros::NodeHandle nh_priv):
        _goal_received(false),
        _goal_reached(false),
        _path_computed(false),
        
        _throttle (0.0),
        _w (0.0),
        _speed(0.0)
    {

    //Parameters for control loop
    nh_priv.param("num_threads", num_threads_, 2); // number of threads for this ROS node
    nh_priv.param("pub_twist_cmd", is_pub_twist_, true);
    nh_priv.param("debug_level", debug_level_, 0);
    nh_priv.param("delay_mode", _delay_mode, true);

    nh_priv.param("max_speed", _max_speed, 2.50); // [m/s]
    nh_priv.param("waypoints_dist", _waypoint_dist_, -1.0); // [m]
    nh_priv.param("path_length", _pathLength, 5.0); // [m]
    nh_priv.param("goal_tolerance", goal_tolerance_, 0.5); // [m]
    nh_priv.param("controller_freq", _controller_freq, 10); // [Hz]
    //pn.param("vehicle_Lf", _Lf, 0.290); // distance between the front of the vehicle and its center of gravity
    _dt = double(1.0/_controller_freq); // time step duration dt in s 

    nh_priv.param("inplace_rotate_vel", inplace_rotate_vel_, 0.15);

    //Parameter for MPC solver
    nh_priv.param("mpc_steps", _mpc_steps, 10.0);

    nh_priv.param("mpc_ref_cte", _ref_cte, 0.0);
    nh_priv.param("mpc_ref_vel", _ref_vel, 3.0);
    nh_priv.param("mpc_ref_etheta", _ref_etheta, 0.0);

    nh_priv.param("mpc_w_cte", _w_cte, 5000.0);
    nh_priv.param("mpc_w_etheta", _w_etheta, 5000.0);
    nh_priv.param("mpc_w_vel", _w_vel, 1.0);
    nh_priv.param("mpc_w_angvel", _w_angvel, 100.0);
    nh_priv.param("mpc_w_angvel_d", _w_angvel_d, 10.0);
    nh_priv.param("mpc_w_accel", _w_accel, 50.0);
    nh_priv.param("mpc_w_accel_d", _w_accel_d, 10.0);

    nh_priv.param("mpc_max_angvel", _max_angvel, 3.0); // Maximal angvel radian (~30 deg)
    nh_priv.param("mpc_max_throttle", _max_throttle, 1.0); // Maximal throttle accel
    nh_priv.param("mpc_bound_value", _bound_value, 1.0e3); // Bound value for other variables

    //Parameter for topics & Frame name
    nh_priv.param<std::string>("odom_topic"        , odom_topic_        , "odom" );
    nh_priv.param<std::string>("path_planner_topic", path_topic_, "/move_base/TrajectoryPlannerROS/global_plan" );
    nh_priv.param<std::string>("goal_topic", goal_topic_, "/move_base_simple/goal" );
    nh_priv.param<std::string>("control_topic", cmd_topic_, "/cmd_vel" );
    
    nh_priv.param<std::string>("map_frame" , map_frame_, "map" ); //*****for mpc, "odom"
    nh_priv.param<std::string>("odom_frame", odom_frame_, "odom");
    nh_priv.param<std::string>("car_frame" , car_frame_, "base_footprint" );

    //Display the parameters
    if ((debug_level_ >=1) ){
        _printParams();
    }

    //Publishers and Subscribers
    odom_sub_               = nh.subscribe( odom_topic_,    1,  &MPCNode::odomCB, this);
    path_planner_sub_       = nh.subscribe( path_topic_,    1,  &MPCNode::pathCB, this);
    
    goal_sub_               = nh.subscribe( goal_topic_,    1,  &MPCNode::goalCB, this);

    
    mpc_ref_pub_            = nh.advertise<nav_msgs::Path>("/mpc_reference", 5); // reference path for MPC ///mpc_reference 
    mpc_traj_pub_            = nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1);// MPC trajectory output
    //_pub_ackermann = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);
    
    if(is_pub_twist_)
        twist_cmd_pub_      = nh.advertise<geometry_msgs::Twist>(cmd_topic_, 5); //for stage (Ackermann msg non-supported)
        twist_stmp_pub_ = nh.advertise<geometry_msgs::TwistStamped>(cmd_topic_+ "_stmp", 15);
    
    //Timer
    _timer1 = nh.createTimer(ros::Duration(_dt), &MPCNode::controlLoopCB, this); // 10Hz //*****mpc

    

    _onetime_noti = false;
    
    //_ackermann_msg = ackermann_msgs::AckermannDriveStamped();
    _twist_msg = geometry_msgs::Twist();
    _mpc_traj = nav_msgs::Path();

    //Init parameters for MPC object
    _mpc_params["DT"] = _dt;

    _mpc_params["STEPS"]    = _mpc_steps;

    _mpc_params["REF_CTE"]  = _ref_cte;
    _mpc_params["REF_ETHETA"] = _ref_etheta;
    _mpc_params["REF_V"]    = _ref_vel;

    _mpc_params["W_CTE"]    = _w_cte;
    _mpc_params["W_EPSI"]   = _w_etheta;
    _mpc_params["W_V"]      = _w_vel;
    _mpc_params["W_ANGVEL"]  = _w_angvel;
    _mpc_params["W_A"]      = _w_accel;
    _mpc_params["W_DANGVEL"] = _w_angvel_d;
    _mpc_params["W_DA"]     = _w_accel_d;

    _mpc_params["ANGVEL"]   = _max_angvel;
    _mpc_params["MAXTHR"]   = _max_throttle;
    _mpc_params["BOUND"]    = _bound_value;
    
    _mpc_params["DEBUG_LEVEL"]= debug_level_;

    mpc_solver_ = MPC(_mpc_params);
}




// Evaluate a polynomial.
double MPCNode::_polyeval(Eigen::VectorXd coeffs, double x) 
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd MPCNode::_polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++) 
    {
        for (int i = 0; i < order; i++) 
            A(j, i + 1) = A(j, i) * xvals(j);
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}


double MPCNode::_getYaw(const geometry_msgs::Pose&pose){
    tf::Pose tf_pose;
    tf::poseMsgToTF(pose, tf_pose);
    return tf::getYaw(tf_pose.getRotation());
}


void MPCNode::_printParams(){
    cout << "[MPC_Node::MPC] pub_twist_cmd : "   << is_pub_twist_ << endl;
    cout << "[MPC_Node::MPC] delay_mode    : "   << _delay_mode << endl;

    cout << "[MPC_Node::MPC] mpc_dt        : "   << _dt << endl;

    cout << "[MPC_Node::MPC] mpc_steps     : "   << _mpc_steps << endl;

    cout << "[MPC_Node::MPC] inplace_rotate_vel: "   << inplace_rotate_vel_ << endl;

    cout << "[MPC_Node::MPC] mpc_ref_cte   : "   << _ref_cte << endl;
    cout << "[MPC_Node::MPC] mpc_ref_vel   : "   << _ref_vel << endl;
    cout << "[MPC_Node::MPC] mpc_ref_etheta: "   << _ref_etheta << endl;

    cout << "[MPC_Node::MPC] mpc_w_cte      : "  << _w_cte << endl;
    cout << "[MPC_Node::MPC] mpc_w_etheta   : "  << _w_etheta << endl;
    cout << "[MPC_Node::MPC] mpc_w_vel      : "  << _w_vel << endl;
    cout << "[MPC_Node::MPC] mpc_w_angvel  : "  << _w_angvel << endl;
    cout << "[MPC_Node::MPC] mpc_w_angvel_d : "  << _w_angvel_d << endl;
    cout << "[MPC_Node::MPC] mpc_w_accel    : "  << _w_accel << endl;
    cout << "[MPC_Node::MPC] mpc_w_accel_d  : "  << _w_accel_d << endl;

    cout << "[MPC_Node::MPC] mpc_max_angvel  : "  << _max_angvel << endl;
    cout << "[MPC_Node::MPC] mpc_max_throttle: "  << _max_throttle << endl;
    cout << "[MPC_Node::MPC] mpc_bound_value : "  << _bound_value << endl;
}


// CallBack: Update goal status
void MPCNode::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg){
    
    _goal_pos = *goalMsg;

    _goal_received = true;
    _goal_reached = false;
    _onetime_noti = true;

    if ((debug_level_ >=1)){
        cout << "[MPC_Node::goalCB] Goal Received\n";
        std::printf("Moving: (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)\n",
                                _odom.pose.pose.position.x,
                                _odom.pose.pose.position.y,
                                _getYaw(_odom.pose.pose), 
                                _goal_pos.pose.position.x,
                                _goal_pos.pose.position.y,
                                _getYaw(_goal_pos.pose));
    }
}



/*
*   \brief Update Odom & compare reached location
*/
void MPCNode::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    
    _odom = *odomMsg;
    // if(debug_level_ >=3){
    //     ROS_INFO("[MPCNode::odomCB] {x,y,theta}={%.3f, %.3f, %.3f}", _odom.pose.pose.position.x,
    //                                                     _odom.pose.pose.position.y,
    //                                                     _getYaw(_odom.pose.pose));
    // }

    if(_goal_received)
    {   // Compute distance to goal and stop if reached [< tolerance]
        double car2goal_x = _goal_pos.pose.position.x - _odom.pose.pose.position.x;
        double car2goal_y = _goal_pos.pose.position.y - _odom.pose.pose.position.y;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        
        if(dist2goal < goal_tolerance_)
        {   
            // Switch flags
            _goal_received = false;
            _goal_reached = true;
            _path_computed = false;
            _onetime_noti = true;

            // Noti
            if(debug_level_ >=1){
                ROS_INFO("[MPCNode] Goal Reached ! dist2goal=%.3f", dist2goal);
                ROS_INFO("[MPCNode] _goal_received=%d, _goal_reached=%d, _path_computed=%d", 
                                    _goal_received, _goal_reached, _path_computed);
            }
            
        }
    }
    
}



// CallBack: Update path waypoints (conversion to odom frame)
void MPCNode::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    // ROS_INFO("[MPC_Node::pathCB] path received!!!");
    // ROS_INFO("[MPC_Node::pathCB], pathMsg size= %d", (int)pathMsg->poses.size());
    
    if(_goal_received && !_goal_reached)
    {   
        if ( debug_level_ >=2 ){
            cout << "[MPC_Node::pathCB] PathCB compute" << endl;
        }

        // Initiate odom_path to store local_trajectory
        nav_msgs::Path odom_path = nav_msgs::Path();
        try {
            double total_length = 0.0;
            int sampling = _downSampling;

            //find waypoints distance
            if(_waypoint_dist_ <=0.0)
            {        
                double dx = pathMsg->poses[1].pose.position.x - pathMsg->poses[0].pose.position.x;
                double dy = pathMsg->poses[1].pose.position.y - pathMsg->poses[0].pose.position.y;
                _waypoint_dist_ = sqrt(dx*dx + dy*dy);
                _downSampling = int(_pathLength/10.0/_waypoint_dist_);
            }            

            // Cut and downsampling the path
            for(int i =0; i< pathMsg->poses.size(); i++)
            {
                if(total_length > _pathLength)
                    break;

                if(sampling == _downSampling)
                {   
                    geometry_msgs::PoseStamped tempPose;
                    _tf_listener.transformPose(odom_frame_, ros::Time(0) , pathMsg->poses[i], map_frame_, tempPose);                     
                    odom_path.poses.push_back(tempPose);  
                    sampling = 0;
                }
                total_length = total_length + _waypoint_dist_; 
                sampling = sampling + 1;  
            }
           
            if(odom_path.poses.size() >= 6 )
            {
                _odom_path = odom_path; // Path waypoints in odom frame
                _path_computed = true;
                // publish odom path
                odom_path.header.frame_id = odom_frame_;
                odom_path.header.stamp = ros::Time::now();
                mpc_ref_pub_.publish(odom_path);
                if ((debug_level_ >=2)){
                    cout << "[MPC_Node::pathCB] Generated path!!!" << endl;
                }
            }
            else{   
                if ((debug_level_ >=2)){
                    cout << "[MPC_Node::pathCB] Failed to generate path!!!" << endl;
                }
                _waypoint_dist_ = -1;
            }
            //DEBUG            
            //cout << endl << "N: " << odom_path.poses.size() << endl 
            //<<  "Car path[0]: " << odom_path.poses[0];
            // << ", path[N]: " << _odom_path.poses[_odom_path.poses.size()-1] << endl;
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("[MPC_Node::pathCB] PathCB condition ERROR");
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    
}



void MPCNode::_mpcCompute(){
    nav_msgs::Odometry odom = _odom; 
    nav_msgs::Path odom_path = _odom_path;   


    // X = [x, y, theta, v]
    const double px     = odom.pose.pose.position.x; //pose: odom frame
    const double py     = odom.pose.pose.position.y;
    const double v      = odom.twist.twist.linear.x; //twist: body fixed frame
    const double theta  = _getYaw(odom.pose.pose);
    
    // U = [w, throttle]
    const double w = _w; // steering -> w
    //const double steering = _steering;  // radian
    const double throttle = _throttle; // accel: >0; brake: <0

    if ((debug_level_ >=1)){
        printf("[MPC_Node::_mpcCompute] odom {%.3f, %.3f, %.3f, %.3f}\n",odom.pose.pose.orientation.x,
                                                                        odom.pose.pose.orientation.y,
                                                                        odom.pose.pose.orientation.z,
                                                                        odom.pose.pose.orientation.w );
        printf("[MPC_Node::_mpcCompute] {x, y, v, theta} =  {%.3f, %.3f, %.3f, %.3f}\n",px, py, v, theta );
    }


    const double dt = _dt;
    //const double Lf = _Lf;

    // Waypoints related parameters
    const int N = odom_path.poses.size(); // Number of waypoints
    const double costheta = cos(theta);
    const double sintheta = sin(theta);

    // Convert to the vehicle coordinate system
    VectorXd x_veh(N);
    VectorXd y_veh(N);
    for(int i = 0; i < N; i++) 
    {
        const double dx = odom_path.poses[i].pose.position.x - px;
        const double dy = odom_path.poses[i].pose.position.y - py;
        x_veh[i] = dx * costheta + dy * sintheta;
        y_veh[i] = dy * costheta - dx * sintheta;
    }
    
    // polinomial trajectory from waypoints
    auto coeffs         = _polyfit(x_veh, y_veh, 3); 

    const double cte    = _polyeval(coeffs, 0.0);
    const double etheta = atan(coeffs[1]);

    VectorXd state(6);
    if(_delay_mode)
    {
        // predict vehicle state at the actual moment of control [t + dt]
        const double px_act     = v * dt;
        const double py_act     = 0;
        const double v_act      = v + throttle * dt;    //v = v + a * dt
        const double theta_act  = w * dt;               //(steering) theta_act = v * steering * dt / Lf;
        
        const double cte_act = cte + v * sin(etheta) * dt;
        const double etheta_act = etheta - theta_act;  
        
        state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
    }
    else
    {
        state << 0, 0, 0, v, cte, etheta;
    }
    
    // Start solving MPC
    // ros::Time begin = ros::Time::now();
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    vector<double> mpc_results = mpc_solver_.Solve(state, coeffs);
    // End solving MPC
    end = std::chrono::system_clock::now();
    // ros::Time end = ros::Time::now();

    // double solving_time = (end.sec - begin.sec) + (end.nsec - begin.nsec)/1000000000.0;
    std::chrono::duration<double> solving_time = end - start;
    printf("[MPCNode::_mpcCompute] Solving time: %.6f\n", solving_time.count());


    // MPC result (all described in car frame), output = (acceleration, w)        
    _w          = mpc_results[0];           // angular velocity [radian/s]
    _throttle   = mpc_results[1];           // acceleration     [m/s^2]
    _speed      = v + _throttle*dt;         // speed            [m/s]
    
    _speed = (_speed >= _max_speed)? _max_speed: _speed;
    _speed = (_speed <= 0.0)        ? 0.0       :_speed;
    
    

    if((debug_level_ >=2))
    {
        cout << "[MPC_Node::_mpcCompute] theta    : " << theta << endl;
        cout << "[MPC_Node::_mpcCompute] v        : " << v << endl;
        cout << "[MPC_Node::_mpcCompute] coeffs   : " << coeffs << endl;
        cout << "[MPC_Node::_mpcCompute] _w       : " << _w << endl;
        cout << "[MPC_Node::_mpcCompute] _throttle: " << _throttle << endl;
        cout << "[MPC_Node::_mpcCompute] _speed   : " << _speed << endl;
    }
    
}

void MPCNode::_publishTwist(double speed, double w){

    // Twist
    _twist_msg.linear.x  = speed; 
    _twist_msg.angular.z = w;
    twist_cmd_pub_.publish(_twist_msg);

    // TwistStamped
    _twist_stmp_msg.twist = _twist_msg;
    _twist_stmp_msg.header.frame_id = "cmd_vel_stamped";
    _twist_stmp_msg.header.stamp = ros::Time::now();
    twist_stmp_pub_.publish(_twist_stmp_msg);
}


// Timer: Control Loop (closed loop nonlinear MPC)
void MPCNode::controlLoopCB(const ros::TimerEvent&)
{          
    if(_goal_received && !_goal_reached && _path_computed ) //received goal & goal not reached    
    {   
        
        double angle = _getYaw(_odom_path.poses[0].pose);
        double theta = _getYaw(_odom.pose.pose);
        double angle_err = _fix_angle(angle - theta);
        
        if ((debug_level_ >=1)){
            printf("[MPC_Node::controlLoopCB] angle:theta {%.3f: %.3f} err=%.3f.\n", angle, theta, angle_err);
        }
        // WARN
        if ( abs(angle_err) > 1.0 ){
            // Inplace rotation
            _speed = 0.0;
            _w = inplace_rotate_vel_*(angle_err)/abs(angle_err) ;
        }
        else{
            // Solve MPC
            _mpcCompute();
            
            
            // Display the MPC predicted trajectory
            _mpc_traj = nav_msgs::Path();
            _mpc_traj.header.frame_id = car_frame_; // points in car coordinate        
            _mpc_traj.header.stamp = ros::Time::now();
            for(int i=0; i<mpc_solver_.mpc_x.size(); i++)
            {
                geometry_msgs::PoseStamped tempPose;
                tempPose.header = _mpc_traj.header;
                tempPose.pose.position.x = mpc_solver_.mpc_x[i];
                tempPose.pose.position.y = mpc_solver_.mpc_y[i];
                tempPose.pose.orientation.w = 1.0;
                _mpc_traj.poses.push_back(tempPose); 
            }     
            // publish the mpc trajectory
            mpc_traj_pub_.publish(_mpc_traj);

        }

    }
    else
    {   
        // Zero command
        _throttle   = 0.0;
        _speed      = 0.0;
        _w          = 0.0;

       
        if(_goal_reached && !_goal_received && _onetime_noti)
        {
            ROS_INFO("[MPC_Node::controlLoopCB] Goal Reached!\n\n\n");
            _onetime_noti = false;
        }
    }

    // publish general cmd_vel 
    if(is_pub_twist_ && _goal_received &&!_goal_reached)
    {
        if( debug_level_ >=1 )
        {   
            printf("[MPC_Node::controlLoopCB] cmd={%.3f, %.3f}\n", _speed, _w);
        }
        
        _publishTwist(_speed, _w);
    }
    
}



int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "mpc_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    MPCNode mpc_node(nh, nh_priv);
    
    // Use multi threads
    ros::AsyncSpinner spinner( (mpc_node.num_thread()>=2)?mpc_node.num_thread():2 );
    spinner.start();
    
    ros::waitForShutdown();
    return 0;
}
