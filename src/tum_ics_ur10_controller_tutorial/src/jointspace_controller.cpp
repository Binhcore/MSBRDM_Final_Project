#include <tum_ics_ur10_controller_tutorial/jointspace_controller.h>

#include <tum_ics_ur_robot_msgs/ControlData.h>


namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    JointSpaceController::JointSpaceController(double weight, const QString &name) : 
      ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
      startFlag_(false),
      Kp_(Matrix6d::Zero()),
      Kd_(Matrix6d::Zero()),
      Ki_(Matrix6d::Zero()),
      goal_(Vector6d::Zero()),
      totalTime_(100.0),
      DeltaQ_(Vector6d::Zero()),
      DeltaQp_(Vector6d::Zero()),
      model_("ur10_model")


      


    {
      control_data_pub_ = nh_.advertise<tum_ics_ur_robot_msgs::ControlData>("joint_space_control_data", 1);

      // load the initial parameters from the parameter server
      ROS_ERROR_STREAM("Initializing model!");
      if(!model_.initRequest(nh_))
      {
      ROS_ERROR_STREAM("Error: initializing model failed!");
      }
      // Initialize the robots parameter vector
      // Matrix<double, 44,1> theta_;
      theta_ = model_.parameterInitalGuess();

    }

    JointSpaceController::~JointSpaceController()
    {
    }

    void JointSpaceController::setQInit(const JointState &qinit)
    {
      qInit_ = qinit;
    }
    void JointSpaceController::setQHome(const JointState &qhome)
    {
      qHome_ = qhome;
    }
    void JointSpaceController::setQPark(const JointState &qpark)
    {
      qPark_ = qpark;
    }

    bool JointSpaceController::init()
    {
      ROS_WARN_STREAM("JointSpaceController::init: loading parameters");

      std::vector<double> vec;

      // check namespace
      std::string ns = "~jointspace_controller";
      if (!ros::param::has(ns))
      {
        ROS_ERROR_STREAM("JointSpaceController init(): Control gains not defined in:" << ns);
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

    bool JointSpaceController::start()
    {
      ROS_WARN_STREAM("JointSpaceController::start");
      return true;
    }

    Vector6d JointSpaceController::update(const RobotTime &time, const JointState &current)
    {
      if (!startFlag_)
      {
        qStart_ = current.q;
        ROS_WARN_STREAM("START [DEG]: \n" << qStart_.transpose());
        startFlag_ = true; std::vector<double> vec;
      }

      /*<ros::param::get("/jointspace_controller/jointspace_controller/goal", vec );
      vec[3] = 90;
      ros::param::set("/jointspace_controller/jointspace_controller/goal", vec );*/


      // broadcast TFs
      model_.broadcastFrames(current.q, ros::Time::now()); 

      // control torque
      Vector6d tau = Vector6d::Zero();
      

      // poly spline, vQd = {pos, vel, acc}
      VVector6d vQd;
      vQd = getJointPVT5(qStart_, goal_, time.tD(), totalTime_);

      // errors
      DeltaQ_ = current.q - vQd[0];
      DeltaQp_ = current.qp - vQd[1];

      // reference
      JointState js_r;
      js_r = current;
      js_r.qp = vQd[1] - Kp_ * DeltaQ_;
      js_r.qpp = vQd[2] - Kp_ * DeltaQp_;

      // ROS_WARN_STREAM("POSITION: \n" << vQd[0]);

      
      /*
      cc::HomogeneousTransformation T_tool_0 = model_.T_tool_0(current.q);
      cc::Rotation3 R_tool_0 = T_tool_0.orientation();
      cc::LinearPosition X_tool_0 = T_tool_0.position();
      Eigen::Quaterniond Q_start(R_tool_0);
      ROS_WARN_STREAM("X_tool_0: \n" << X_tool_0);
      ROS_WARN_STREAM("Q_vec: \n" << Q_start.vec());
      ROS_WARN_STREAM("Q_w: \n" << Q_start.w());
      */
      
      
      
      
      // load robot regressor
      Yr_ = model_.regressor(vQd[1], vQd[2], js_r.qp, js_r.qpp);

      //ROS_WARN_STREAM("Regressor: \n" << Yr_);

      

      // error between reference and current joint velocity
      Vector6d Sq = current.qp - js_r.qp;


      /*
      ros::Time prev_time;
      ros::Time current_time = ros::Time::now();
      ros::Duration dt = (current_time - prev_time); // delta t approx. 0.002s
      prev_time = current_time; 
      */
      
      // gamma and delta_t for learning rate
      cc::Scalar gamma_inv = 1.0 ;
      cc::Scalar dt = 0.002; // approx. 0.002s
      /*
      ros::Time begin = ros::Time::now();
      ros::Duration dt = ros::Time::now() - begin;
      */
     /*
      ros::Time time = ros::Time::now();
      double dt = (time- prev_time).toSec();
      prev_time = time;
      */

      // parameter estimation error
      if (Sq.norm() > 1e-6 )
      {
        theta_ -= gamma_inv*Yr_.transpose()*Sq*dt;
        //ROS_WARN_STREAM("Parameter Vector: \n" << theta_);

      } 

      tau = -Kd_ * Sq + Yr_*theta_;
      



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

      //ROS_WARN_STREAM("tau=" << tau.transpose());
      // tau.setZero()
      return tau;
      
    }

    bool JointSpaceController::stop()
    {
      return true;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
