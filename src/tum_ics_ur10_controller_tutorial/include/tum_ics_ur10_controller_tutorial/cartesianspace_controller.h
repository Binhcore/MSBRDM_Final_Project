#ifndef UR_ROBOT_LLI_CARTESIANSPACE_CONTROLLER_H
#define UR_ROBOT_LLI_CARTESIANSPACE_CONTROLLER_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <ur_model/ur_model.h>
#include <control_core/trajectory/state_trajectories.h>
#include <control_core/types.h>
#include <control_core/trajectory/polynomial_trajectories.h>


namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    class CartesianSpaceController : public ControlEffort
    {
    private:
      ur::URModel model_; // Declare object "model"
      ur::URModel::Parameters theta_; // Declare parameter theta_ from class URModel
      ur::URModel::Regressor Yr_; // Declare robot regressor from URModel


      bool startFlag_;
      bool is_cartesian_flag_;
      bool is_circular_flag_;

      Vector6d qStart_;
      JointState qInit_;
      JointState qHome_;
      JointState qPark_;

      ros::NodeHandle nh_;
      ros::Publisher control_data_pub_;

      Matrix6d Kp_;
      Matrix6d Kd_;
      Matrix6d Ki_;
      Vector6d goal_;
      double totalTime_;

      Vector6d DeltaQ_;
      Vector6d DeltaQp_;

      cc::CartesianStateSlerpTrajectory* CartesianTraj_; // pointer for cartesian trajectory
      ros::Time cartesian_traj_start_time_; // time for trajectory

      double X_circ;
      double Y_circ;
      double PI;


    public:
      CartesianSpaceController(double weight = 1.0, const QString &name = "SimpleEffortCtrl");

      ~CartesianSpaceController();

      void setQInit(const JointState &qinit);

      void setQHome(const JointState &qhome);

      void setQPark(const JointState &qpark);

    private:
      bool init();

      bool start();

      /**
       * @brief update the controller
       * 
       * @param time holds the current robot time
       * @param current current jointstate q, qP of the real robot
       * @return Vector6d next torque command
       */
      Vector6d update(const RobotTime &time, const JointState &current);

      Vector6d updateCartesian(const RobotTime &time, const JointState &current);
      Vector6d updateJoint(const RobotTime &time, const JointState &current);

      bool stop();
    };

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli

#endif // UR_ROBOT_LLI_CARTESIANSPACE_CONTROLLER_H
