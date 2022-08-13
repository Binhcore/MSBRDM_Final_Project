#ifndef UR_ROBOT_LLI_JOINTSPACE_CONTROLLER_H
#define UR_ROBOT_LLI_JOINTSPACE_CONTROLLER_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <ur_model/ur_model.h>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    class JointSpaceController : public ControlEffort
    {
    private:
      ur::URModel model_; // Declare object "model"
      ur::URModel::Parameters theta_; // Declare parameter theta_ from class URModel
      ur::URModel::Regressor Yr_; // Declare robot regressor from URModel


      bool startFlag_;

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

    public:
      JointSpaceController(double weight = 1.0, const QString &name = "SimpleEffortCtrl");

      ~JointSpaceController();

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

      bool stop();
    };

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli

#endif // UR_ROBOT_LLI_JOINTSPACE_CONTROLLER_H
