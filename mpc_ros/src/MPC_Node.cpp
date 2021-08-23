#include <iostream>
#include <map>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
// #include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include "MPC.h"
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
        MPCNode();
        int get_thread_numbers();
        
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _sub_odom, _sub_gen_path, _sub_path, _sub_goal, _sub_amcl;
        ros::Publisher _pub_globalpath,_pub_odompath, _pub_twist, _pub_mpctraj;
        ros::Publisher _pub_ackermann;
        ros::Timer _timer1;
        tf::TransformListener _tf_listener;

        geometry_msgs::Point _goal_pos;
        nav_msgs::Odometry _odom;
        nav_msgs::Path _odom_path, _mpc_traj; 
        //ackermann_msgs::AckermannDriveStamped _ackermann_msg;
        geometry_msgs::Twist _twist_msg;

        string _globalPath_topic, _goal_topic;
        string _map_frame, _odom_frame, _car_frame;

        MPC _mpc;
        map<string, double> _mpc_params;
        double _mpc_steps, _ref_cte, _ref_etheta, _ref_vel, _w_cte, _w_etheta, _w_vel, 
               _w_angvel, _w_accel, _w_angvel_d, _w_accel_d, _max_angvel, _max_throttle, _bound_value;

        //double _Lf; 
        double _dt, _w, _throttle, _speed, _max_speed;
        double _pathLength, _goalRadius, _waypointsDist;
        int _controller_freq, _downSampling, _thread_numbers, _debug_level;
        bool _goal_received, _goal_reached, _path_computed, _pub_twist_flag, _debug_info, _delay_mode, _onetime_noti;

        double polyeval(Eigen::VectorXd coeffs, double x);
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void desiredPathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
        void controlLoopCB(const ros::TimerEvent&);
        void makeGlobalPath(const nav_msgs::Odometry odomMsg);

        //For making global planner
        nav_msgs::Path _gen_path;

}; // end of class


MPCNode::MPCNode()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Parameters for control loop
    pn.param("thread_numbers", _thread_numbers, 2); // number of threads for this ROS node
    pn.param("pub_twist_cmd", _pub_twist_flag, true);
    pn.param("debug_info", _debug_info, true);
    pn.param("debug_level", _debug_level, 0);
    pn.param("delay_mode", _delay_mode, true);
    pn.param("max_speed", _max_speed, 2.50); // unit: m/s
    pn.param("waypoints_dist", _waypointsDist, -1.0); // unit: m
    pn.param("path_length", _pathLength, 8.0); // unit: m
    pn.param("goal_radius", _goalRadius, 0.5); // unit: m
    pn.param("controller_freq", _controller_freq, 10);
    //pn.param("vehicle_Lf", _Lf, 0.290); // distance between the front of the vehicle and its center of gravity
    _dt = double(1.0/_controller_freq); // time step duration dt in s 

    //Parameter for MPC solver
    pn.param("mpc_steps", _mpc_steps, 20.0);

    pn.param("mpc_ref_cte", _ref_cte, 0.0);
    pn.param("mpc_ref_vel", _ref_vel, 3.0);
    pn.param("mpc_ref_etheta", _ref_etheta, 0.0);

    pn.param("mpc_w_cte", _w_cte, 5000.0);
    pn.param("mpc_w_etheta", _w_etheta, 5000.0);
    pn.param("mpc_w_vel", _w_vel, 1.0);
    pn.param("mpc_w_angvel", _w_angvel, 100.0);
    pn.param("mpc_w_angvel_d", _w_angvel_d, 10.0);
    pn.param("mpc_w_accel", _w_accel, 50.0);
    pn.param("mpc_w_accel_d", _w_accel_d, 10.0);

    pn.param("mpc_max_angvel", _max_angvel, 3.0); // Maximal angvel radian (~30 deg)
    pn.param("mpc_max_throttle", _max_throttle, 1.0); // Maximal throttle accel
    pn.param("mpc_bound_value", _bound_value, 1.0e3); // Bound value for other variables

    //Parameter for topics & Frame name
    pn.param<std::string>("global_path_topic", _globalPath_topic, "/move_base/TrajectoryPlannerROS/global_plan" );
    pn.param<std::string>("goal_topic", _goal_topic, "/move_base_simple/goal" );
    pn.param<std::string>("map_frame", _map_frame, "map" ); //*****for mpc, "odom"
    pn.param<std::string>("odom_frame", _odom_frame, "odom");
    pn.param<std::string>("car_frame", _car_frame, "base_footprint" );

    //Display the parameters
    if (_debug_info && (_debug_level >=1) ){
        cout << "[MPC_Node::MPC] pub_twist_cmd : "   << _pub_twist_flag << endl;
        cout << "[MPC_Node::MPC] delay_mode    : "   << _delay_mode << endl;

        cout << "\n[MPC_Node::MPC] mpc_dt        : "   << _dt << endl;

        cout << "\n[MPC_Node::MPC] mpc_steps     : "   << _mpc_steps << endl;

        cout << "\n[MPC_Node::MPC] mpc_ref_cte   : "   << _ref_cte << endl;
        cout << "[MPC_Node::MPC] mpc_ref_vel   : "   << _ref_vel << endl;
        cout << "[MPC_Node::MPC] mpc_ref_etheta: "   << _ref_etheta << endl;

        cout << "\n[MPC_Node::MPC] mpc_w_cte      : "  << _w_cte << endl;
        cout << "[MPC_Node::MPC] mpc_w_etheta   : "  << _w_etheta << endl;
        cout << "[MPC_Node::MPC] mpc_w_vel      : "  << _w_vel << endl;
        cout << "[MPC_Node::MPC] mpc_w_angvel  : "  << _w_angvel << endl;
        cout << "[MPC_Node::MPC] mpc_w_angvel_d : "  << _w_angvel_d << endl;
        cout << "[MPC_Node::MPC] mpc_w_accel    : "  << _w_accel << endl;
        cout << "[MPC_Node::MPC] mpc_w_accel_d  : "  << _w_accel_d << endl;

        cout << "\n[MPC_Node::MPC] mpc_max_angvel  : "  << _max_angvel << endl;
        cout << "[MPC_Node::MPC] mpc_max_throttle: "  << _max_throttle << endl;
        cout << "[MPC_Node::MPC] mpc_bound_value : "  << _bound_value << endl;
    }

    //Publishers and Subscribers
    _sub_odom   = _nh.subscribe("/odom", 1, &MPCNode::odomCB, this);
    _sub_path   = _nh.subscribe( _globalPath_topic, 1, &MPCNode::pathCB, this);
    _sub_gen_path   = _nh.subscribe( "desired_path", 1, &MPCNode::desiredPathCB, this);
    _sub_goal   = _nh.subscribe( _goal_topic, 1, &MPCNode::goalCB, this);
    _sub_amcl   = _nh.subscribe("/amcl_pose", 5, &MPCNode::amclCB, this);
    _pub_globalpath  = _nh.advertise<nav_msgs::Path>("/global_path", 1); // Global path generated from another source
    _pub_odompath  = _nh.advertise<nav_msgs::Path>("/mpc_reference", 1); // reference path for MPC ///mpc_reference 
    _pub_mpctraj   = _nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1);// MPC trajectory output
    //_pub_ackermann = _nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);
    if(_pub_twist_flag)
        _pub_twist = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //for stage (Ackermann msg non-supported)
    
    //Timer
    _timer1 = _nh.createTimer(ros::Duration((1.0)/_controller_freq), &MPCNode::controlLoopCB, this); // 10Hz //*****mpc

    //Init variables
    _goal_received = false;
    _goal_reached  = false;
    _path_computed = false;
    _throttle = 0.0; 
    _w = 0.0;
    _speed = 0.0;

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

    _mpc_params["DEBUGGING"]= _debug_info;
    _mpc_params["DEBUG_LEVEL"]= _debug_level;

    _mpc.LoadParams(_mpc_params);
}


// Public: return _thread_numbers
int MPCNode::get_thread_numbers()
{
    return _thread_numbers;
}


// Evaluate a polynomial.
double MPCNode::polyeval(Eigen::VectorXd coeffs, double x) 
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
Eigen::VectorXd MPCNode::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
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

// CallBack: Update odometry
void MPCNode::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    _odom = *odomMsg;
    if (_debug_info && (_debug_level >=3)){
        std::cout << "Update location: \n" << _odom << std::endl;
    }
    
}

void MPCNode::makeGlobalPath(const nav_msgs::Odometry odomMsg)
{

}

// CallBack: Update generated path (conversion to odom frame)
void MPCNode::desiredPathCB(const nav_msgs::Path::ConstPtr& totalPathMsg)
{
}

// CallBack: Update path waypoints (conversion to odom frame)
void MPCNode::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    // ROS_INFO("[MPC_Node::pathCB] path received!!!");
    // ROS_INFO("[MPC_Node::pathCB], pathMsg size= %d", (int)pathMsg->poses.size());
    
    if(_goal_received && !_goal_reached)
    {   
        if (_debug_info && (_debug_level >=2)){
            cout << "[MPC_Node::pathCB] PathCB compute" << endl;
        }

        // Initiate odom_path to store local_trajectory
        nav_msgs::Path odom_path = nav_msgs::Path();
        try
        {
            double total_length = 0.0;
            int sampling = _downSampling;

            //find waypoints distance
            if(_waypointsDist <=0.0)
            {        
                double dx = pathMsg->poses[1].pose.position.x - pathMsg->poses[0].pose.position.x;
                double dy = pathMsg->poses[1].pose.position.y - pathMsg->poses[0].pose.position.y;
                _waypointsDist = sqrt(dx*dx + dy*dy);
                _downSampling = int(_pathLength/10.0/_waypointsDist);
            }            

            // Cut and downsampling the path
            for(int i =0; i< pathMsg->poses.size(); i++)
            {
                if(total_length > _pathLength)
                    break;

                if(sampling == _downSampling)
                {   
                    geometry_msgs::PoseStamped tempPose;
                    _tf_listener.transformPose(_odom_frame, ros::Time(0) , pathMsg->poses[i], _map_frame, tempPose);                     
                    odom_path.poses.push_back(tempPose);  
                    sampling = 0;
                }
                total_length = total_length + _waypointsDist; 
                sampling = sampling + 1;  
            }
           
            if(odom_path.poses.size() >= 6 )
            {
                _odom_path = odom_path; // Path waypoints in odom frame
                _path_computed = true;
                // publish odom path
                odom_path.header.frame_id = _odom_frame;
                odom_path.header.stamp = ros::Time::now();
                _pub_odompath.publish(odom_path);
                if (_debug_info && (_debug_level >=2)){
                    cout << "[MPC_Node::pathCB] Generated path!!!" << endl;
                }
            }
            else{   
                if (_debug_info && (_debug_level >=2)){
                    cout << "[MPC_Node::pathCB] Failed to generate path!!!" << endl;
                }
                _waypointsDist = -1;
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

// CallBack: Update goal status
void MPCNode::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    _goal_pos = goalMsg->pose.position;
    _goal_received = true;
    _goal_reached = false;
    _onetime_noti = true;
    if (_debug_info && (_debug_level >=1)){
        cout << "[MPC_Node::goalCB] Goal Received\n";
        tf::Pose pose;
        tf::poseMsgToTF(_odom.pose.pose, pose);
        std::printf("Moving: (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)\n",
                                _odom.pose.pose.position.x,
                                _odom.pose.pose.position.y,
                                tf::getYaw(pose.getRotation()), 
                                _goal_pos.x,
                                _goal_pos.y,
                                _goal_pos.z);
    }
    
}


// Callback: Check if the car is inside the goal area or not 
void MPCNode::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
{
    if(_goal_received)
    {
        double car2goal_x = _goal_pos.x - amclMsg->pose.pose.position.x;
        double car2goal_y = _goal_pos.y - amclMsg->pose.pose.position.y;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        if(dist2goal < _goalRadius)
        {
            _goal_received = false;
            _goal_reached = true;
            _path_computed = false;
            _onetime_noti = true;
            ROS_INFO("[MPC_Node::amclCB] Goal Reached ! dist2goal=%.3f", dist2goal);
            ROS_INFO("[MPC_Node::amclCB] _goal_received=%d, _goal_reached=%d, _path_computed=%d", _goal_received, _goal_reached, _path_computed);
        }
    }
}


// Timer: Control Loop (closed loop nonlinear MPC)
void MPCNode::controlLoopCB(const ros::TimerEvent&)
{          
    if(_goal_received && !_goal_reached && _path_computed ) //received goal & goal not reached    
    {   
        if (_debug_info && (_debug_level >=1)){
            std::cout << "[MPC_Node::controlLoopCB] Compute control command\n";
        }

        nav_msgs::Odometry odom = _odom; 
        nav_msgs::Path odom_path = _odom_path;   

        // Update system states: X=[x, y, theta, v]
        const double px = odom.pose.pose.position.x; //pose: odom frame
        const double py = odom.pose.pose.position.y;
        tf::Pose pose;
        tf::poseMsgToTF(odom.pose.pose, pose);
        const double theta = tf::getYaw(pose.getRotation());
        const double v = odom.twist.twist.linear.x; //twist: body fixed frame
        // Update system inputs: U=[w, throttle]
        const double w = _w; // steering -> w
        //const double steering = _steering;  // radian
        const double throttle = _throttle; // accel: >0; brake: <0

        if (_debug_info && (_debug_level >=1)){
            cout << "[MPC_Node::controlLoopCB] x    : " << px << endl;
            cout << "[MPC_Node::controlLoopCB] y    : " << py << endl;
            cout << "[MPC_Node::controlLoopCB] v    : " << v << endl;
            cout << "[MPC_Node::controlLoopCB] theta: " << theta << endl;
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
        
        // Fit waypoints
        auto coeffs = polyfit(x_veh, y_veh, 3); 

        const double cte  = polyeval(coeffs, 0.0);
        const double etheta = atan(coeffs[1]);

        VectorXd state(6);
        if(_delay_mode)
        {
            // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
            const double px_act = v * dt;
            const double py_act = 0;
            const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;
            const double v_act = v + throttle * dt; //v = v + a * dt
            
            const double cte_act = cte + v * sin(etheta) * dt;
            const double etheta_act = etheta - theta_act;  
            
            state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
        }
        else
        {
            state << 0, 0, 0, v, cte, etheta;
        }
        
        // Solve MPC Problem
        ros::Time begin = ros::Time::now();
        vector<double> mpc_results = _mpc.Solve(state, coeffs);
        ros::Time end = ros::Time::now();
        uint32_t solving_time = (end.sec * 1000 + end.nsec/1000000) - (begin.nsec/1000000 + begin.sec*1000);
        cout << "[MPCNode::controlLoopCB] Solving time: " << solving_time << endl;
        // std::printf("[MPCNode::controlLoopCB] (%.3f, %.3f) -> (%.3f, %.3f)\n",  odom.pose.pose.position.x,
                                                                                // odom.pose.pose.position.y,
                                                                                // _goal_pos.x,
                                                                                // _goal_pos.y );
        // MPC result (all described in car frame), output = (acceleration, w)        
        _w = mpc_results[0]; // radian/sec, angular velocity
        _throttle = mpc_results[1]; // acceleration
        _speed = v + _throttle*dt;  // speed
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if(_speed <= 0.0)
            _speed = 0.0;

        if(_debug_info && (_debug_level >=2))
        {
            cout << "[MPC_Node::controlLoopCB] theta    : " << theta << endl;
            cout << "[MPC_Node::controlLoopCB] v        : " << v << endl;
            cout << "[MPC_Node::controlLoopCB] coeffs   : " << coeffs << endl;
            cout << "[MPC_Node::controlLoopCB] _w       : " << _w << endl;
            cout << "[MPC_Node::controlLoopCB] _throttle: " << _throttle << endl;
            cout << "[MPC_Node::controlLoopCB] _speed   : " << _speed << endl;
        }

        // Display the MPC predicted trajectory
        _mpc_traj = nav_msgs::Path();
        _mpc_traj.header.frame_id = _car_frame; // points in car coordinate        
        _mpc_traj.header.stamp = ros::Time::now();
        for(int i=0; i<_mpc.mpc_x.size(); i++)
        {
            geometry_msgs::PoseStamped tempPose;
            tempPose.header = _mpc_traj.header;
            tempPose.pose.position.x = _mpc.mpc_x[i];
            tempPose.pose.position.y = _mpc.mpc_y[i];
            tempPose.pose.orientation.w = 1.0;
            _mpc_traj.poses.push_back(tempPose); 
        }     
        // publish the mpc trajectory
        _pub_mpctraj.publish(_mpc_traj);

    }
    else
    {
        _throttle = 0.0;
        _speed = 0.0;
        _w = 0;
        if(_goal_reached && !_goal_received && _onetime_noti)
        {
            ROS_INFO("[MPC_Node::controlLoopCB] Goal Reached!\n\n\n");
            _onetime_noti = false;
        }
    }

    // publish ankermann cmd_vel
    /*
    _ackermann_msg.header.frame_id = _car_frame;
    _ackermann_msg.header.stamp = ros::Time::now();
    _ackermann_msg.drive.steering_angle = _steering;
    _ackermann_msg.drive.speed = _speed;
    _ackermann_msg.drive.acceleration = _throttle;
    _pub_ackermann.publish(_ackermann_msg);        
    */

    // publish general cmd_vel 
    if(_pub_twist_flag)
    {
        if(_debug_info && (_debug_level >=1))
        {
            cout << "[MPC_Node::controlLoopCB] _speed   : " << _speed << endl;
            cout << "[MPC_Node::controlLoopCB] w        : " << _w << endl;
        }
        _twist_msg.linear.x  = _speed; 
        _twist_msg.angular.z = _w;
        _pub_twist.publish(_twist_msg);
    }
    
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "MPC_Node");
    MPCNode mpc_node;

    ROS_INFO("[MPC_Node::main] Waiting for global path msgs ~");
    ros::AsyncSpinner spinner(mpc_node.get_thread_numbers()); // Use multi threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

