#include <tum_ics_ur10_controller_tutorial/cartesianspace_controller.h>

#include <tum_ics_ur_robot_msgs/ControlData.h>

#include <control_core/trajectory/state_trajectories.h>
#include <control_core/types.h>
#include <control_core/trajectory/polynomial_trajectories.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>



namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    CartesianSpaceController::CartesianSpaceController(double weight, const QString &name) : 
      ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
      startFlag_(false),
      Kp_(Matrix6d::Zero()),
      Kd_(Matrix6d::Zero()),
      Ki_(Matrix6d::Zero()),
      goal_(Vector6d::Zero()),
      totalTime_(10.0),
      DeltaQ_(Vector6d::Zero()),
      DeltaQp_(Vector6d::Zero()),
      model_("ur10_model"),
      is_cartesian_flag_(false),
      is_circular_flag_(true)
    {
      control_data_pub_ = nh_.advertise<tum_ics_ur_robot_msgs::ControlData>("joint_space_control_data", 1);

      // load the initial parameters from the parameter server
      ROS_ERROR_STREAM("Initializing model!");
      if(!model_.initRequest(nh_))
      {
      ROS_ERROR_STREAM("Error: initializing model failed!");
      }

      // initialize the robots parameter vector
      theta_ = model_.parameterInitalGuess();

      // generate trajecotry
      cc::CartesianPosition X_start(-0.8063, -0.1639, 0.6296, 0.5056, 0.4943, -0.4943, -0.5056); // start position + quaternion (-0.8339, -0.1639, 0.6140, 0.4989, -0.4989, -0.5011, 0.5011)
      cc::CartesianPosition X_target(-0.9470, -0.1639, 0.3915, 0, 0.0121, 0, 1); // target position + quaternion (-0.9508, -0.1639, 0.3918, 0.0150, 0, 0.9998, 0)
      CartesianTraj_ = new cc::CartesianStateSlerpTrajectory(totalTime_, X_start, X_target);


      // generate circular trajectory
      if (is_circular_flag_)
      { 
        double PI = 3.14159265358979323846;
        double X_circ(0.0);
        double Y_circ(0.0);
        for (double t = 0.0; t <= 2*PI; t = t + 0.001)
        {

        X_circ = -0.9470 + 0.5*cos(t);
        Y_circ = -0.9470 + 0.5*sin(t);
        if (t >= 2*PI)
          t = 0.0;
        }
      } 

    }

    CartesianSpaceController::~CartesianSpaceController()
    {
    }

    void CartesianSpaceController::setQInit(const JointState &qinit)
    {
      qInit_ = qinit;
    }
    void CartesianSpaceController::setQHome(const JointState &qhome)
    {
      qHome_ = qhome;
    }
    void CartesianSpaceController::setQPark(const JointState &qpark)
    {
      qPark_ = qpark;
    }

    bool CartesianSpaceController::init()
    {
      ROS_WARN_STREAM("CartesianSpaceController::init: loading parameters");

      std::vector<double> vec;

      // check namespace
      std::string ns = "~cartesianspace_controller";
      if (!ros::param::has(ns))
      {
        ROS_ERROR_STREAM("CartesianSpaceController init(): Control gains not defined in:" << ns);
        m_error = true;
        return false;
      }

      // D GAINS
      ros::param::get(ns + "/gains_d", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_d: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (size_t i = 0; i < STD_DOF; i++)
      {
        Kd_(i, i) = vec[i];
      }
      ROS_WARN_STREAM("Kd: \n" << Kd_);

      // P GAINS
      ros::param::get(ns + "/gains_p", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++)
      {
        Kp_(i, i) = vec[i] / Kd_(i, i);
      }
      ROS_WARN_STREAM("Kp: \n" << Kp_);

      // GOAL
      ros::param::get(ns + "/goal", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++)
      {
        goal_(i) = vec[i];
      }
      
      // total time
      ros::param::get(ns + "/time", totalTime_);
      if (!(totalTime_ > 0))
      {
        ROS_ERROR_STREAM("totalTime_: is negative:" << totalTime_);
        totalTime_ = 100.0;
      }

      ROS_WARN_STREAM("Goal [DEG]: \n" << goal_.transpose());
      ROS_WARN_STREAM("Total Time [s]: " << totalTime_);
      goal_ = DEG2RAD(goal_);
      ROS_WARN_STREAM("Goal [RAD]: \n" << goal_.transpose());
      return true;
    }

    bool CartesianSpaceController::start()
    {
      ROS_WARN_STREAM("CartesianSpaceController::start");
      return true;
    }


    Vector6d CartesianSpaceController::update(const RobotTime &time, const JointState &current)
    {
      
      if(!is_cartesian_flag_)
      {
        return updateJoint(time, current);
      }

      if (!startFlag_)
      {
        qStart_ = current.q;
        startFlag_ = true; std::vector<double> vec;
        cartesian_traj_start_time_ = ros::Time::now();
        
      }
      
      // broadcast TFs
      model_.broadcastFrames(current.q, ros::Time::now()); 

      // control torque
      Vector6d tau = Vector6d::Zero();
      
       // from jointspace controller
      VVector6d vQd;
      vQd = getJointPVT5(qStart_, goal_, time.tD(), totalTime_);
      
      // get the end effector jacobians wrt {0} frame
      cc::Jacobian J_tool_0 = model_.J_tool_0(current.q);
      cc::Jacobian Jp_tool_0 = model_.Jp_tool_0(current.q, current.qp);
      
      // get the end effector transformation wrt {0} frame
      cc::HomogeneousTransformation T_tool_0 = model_.T_tool_0(current.q);
      cc::Rotation3 R_tool_0 = T_tool_0.orientation();
      cc::LinearPosition X_tool_0 = T_tool_0.position();

      // generate cartesian trajectory
      double elapsed_time = (ros::Time::now() - cartesian_traj_start_time_).toSec();
      cc::CartesianStateSlerpTrajectory::State state = CartesianTraj_->evaluate(elapsed_time);
      cc::LinearPosition Xd = state.X().position(); // (desired) trajectory position
      cc::AngularPosition Od = state.X().orientation(); // (desired) trajectory orientation
      cc::CartesianVelocity Xdp = state.XP(); // (desired) trajectory velocity
      cc::CartesianAcceleration Xdpp = state.XPP(); // (desired) trajectory acceleration

      // position/angular errors
      cc::CartesianVector Delta_X; 
      Delta_X.linear() << X_tool_0 - Xd; // position error between traj and tool in cartesian space
      cc::AngularPosition O_tool_0 = R_tool_0;
      Delta_X.angular() = -cc::logErrorWorld(Od,O_tool_0); // angular error between traj and tool

      // cartesian space references
      Vector6d Xp = J_tool_0*current.qp; // current cartesian velocity tool
      Vector6d Xpp = J_tool_0*current.qpp + Jp_tool_0*current.qp; // current cartesian acceleration tool
      Vector6d Delta_Xp = Xp - Xdp; // cartesian velocity error between traj and tool
      Vector6d Xrp = Xdp - Kp_*Delta_X; // cartesian velocity reference
      Vector6d Xrpp = Xdpp - Kp_*Delta_Xp; // cartesian acceleration reference

      // moore-penrose pseudo inverse with damping 0.05 when SV < 0.5
      cc::Jacobian::Inverse J_pinv = J_tool_0.dampedPinv(0.05, 0.5);

      Vector6d Qrp = J_pinv*Xrp; // joint velocity reference
      Vector6d Qrpp = J_pinv*(Xrpp - Jp_tool_0*Qrp); // joint acceleration reference

      // joint space references
      JointState js_r;
      js_r = current;
      js_r.qp = Qrp;
      js_r.qpp = Qrpp;

      // load robot regressor
      Yr_ = model_.regressor(current.q, current.qp, js_r.qp, js_r.qpp);

      Vector6d Sx = Xp - Xrp; // error between reference and current cartesian velocity
      Vector6d Sq= J_pinv*Sx; // error between reference and current joint velocity

      // gamma and dt as learning rate
      cc::Scalar gamma_inv = 1.0 ;
      cc::Scalar dt = 0.002; // approx. 0.002s

      /*
      // publish cartesian position error
      ros::NodeHandle n;
      ros::Publisher position_error = n.advertise<cc::CartesianVector>("pos_error",1000);

      cc::CartesianVector err;
      err.data = Delta_X;
      position_error.publish(err); 
      */

      // parameter estimation error
      if (Sq.norm() > 1e-6 )
      {
        theta_ -= gamma_inv*Yr_.transpose()*Sq*dt;
      } 

      // control torque
      tau = -Kd_ * Sq + Yr_*theta_;
      ROS_WARN_STREAM("Cartesianspace Control \n");
      
      // publish the ControlData (only for debugging)
      tum_ics_ur_robot_msgs::ControlData msg;
      msg.header.stamp = ros::Time::now();
      msg.time = time.tD();
      for (int i = 0; i < STD_DOF; i++)
      {
        msg.q[i] = current.q(i);
        msg.qp[i] = current.qp(i);
        msg.qpp[i] = current.qpp(i);

        msg.qd[i] = vQd[0](i);
        msg.qpd[i] = vQd[1](i);

        msg.Dq[i] = DeltaQ_(i);
        msg.Dqp[i] = DeltaQp_(i);

        msg.torques[i] = current.tau(i);
      }
      control_data_pub_.publish(msg);

      return tau;
    }


    Vector6d CartesianSpaceController::updateJoint(const RobotTime &time, const JointState &current)
    {
      if (!startFlag_)
      {
        qStart_ = current.q;;
        startFlag_ = true; std::vector<double> vec;
      }

      // broadcast TFs
      model_.broadcastFrames(current.q, ros::Time::now()); 

      // control torque
      Vector6d tau = Vector6d::Zero();

      // poly spline, vQd = {pos, vel, acc}
      VVector6d vQd;
      vQd = getJointPVT5(qStart_, goal_, time.tD(), 7.0);

      // errors
      DeltaQ_ = current.q - vQd[0];
      DeltaQp_ = current.qp - vQd[1];

      // reference
      JointState js_r;
      js_r = current;
      js_r.qp = vQd[1] - Kp_ * DeltaQ_;
      js_r.qpp = vQd[2] - Kp_ * DeltaQp_;

      // get position error of tool and target
      cc::LinearPosition X_joint_end(-0.8063, -0.1639, 0.6296);
      cc::HomogeneousTransformation T_tool_0 = model_.T_tool_0(current.q);
      cc::LinearPosition X_tool_0 = T_tool_0.position();
      cc::Rotation3 R_tool_0 = T_tool_0.orientation();
      Eigen::Quaterniond Q_start(R_tool_0);
      
      cc::LinearPosition pos_err = X_tool_0 - X_joint_end;

      // load robot regressor
      Yr_ = model_.regressor(vQd[1], vQd[2], js_r.qp, js_r.qpp);

      // error between reference and current joint angle
      Vector6d Sq = current.qp - js_r.qp;
      
      // gamma and delta_t for learning rate
      cc::Scalar gamma_inv = 1.0 ;
      cc::Scalar dt = 0.002; // approx. 0.002s

      // parameter estimation error
      if (Sq.norm() > 1e-6 )
      {
        theta_ -= gamma_inv*Yr_.transpose()*Sq*dt;
      } 

      tau = -Kd_ * Sq + Yr_*theta_;
      ROS_WARN_STREAM("JointSpace Control \n");
      //ROS_WARN_STREAM("Position error: \n" <<pos_err.norm());
      //ROS_WARN_STREAM("Velocity error: \n" <<Sq.norm());
  
      
      // publish the ControlData (only for debugging)
      tum_ics_ur_robot_msgs::ControlData msg;
      msg.header.stamp = ros::Time::now();
      msg.time = time.tD();
      for (int i = 0; i < STD_DOF; i++)
      {
        msg.q[i] = current.q(i);
        msg.qp[i] = current.qp(i);
        msg.qpp[i] = current.qpp(i);

        msg.qd[i] = vQd[0](i);
        msg.qpd[i] = vQd[1](i);

        msg.Dq[i] = DeltaQ_(i);
        msg.Dqp[i] = DeltaQp_(i);

        msg.torques[i] = current.tau(i);
      }
      control_data_pub_.publish(msg);

      // State machine switches to Cartesianspace Controller
      if (pos_err.norm() < 1e-3)
      {
      startFlag_ = false;
      is_cartesian_flag_ =true;
      ROS_WARN_STREAM("Joint Target reached \n");
      ROS_WARN_STREAM("Switch to Cartesianspace Controller \n");
      }

      return tau;
      
    }

    bool CartesianSpaceController::stop()
    {
      return true;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
