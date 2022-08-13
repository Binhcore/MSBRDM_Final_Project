#ifndef OBJECT_SERVER_TRAJECTORIES_H_
#define OBJECT_SERVER_TRAJECTORIES_H_

#include <control_core/types.h>
#include <control_core/trajectory/state_trajectories.h>
#include <control_core/trajectory/polynomial_trajectories.h>

/**
 * @brief Create some sample trajectoreis to move objects around
 * 
 */
namespace cc
{
  class CartesianTrajectory
  {
  public:
    CartesianTrajectory(
      cc::Scalar period,
      const cc::LinearPosition &x_i_w) : omega_(2.0 * M_PI / period),
                                         x_i_w_(x_i_w)
    {
    }

    virtual ~CartesianTrajectory()
    {
    }

    virtual cc::CartesianState evaluate(cc::Scalar time) = 0;

    /**
     * @brief convert to ros message type
     * 
     * @param n_steps 
     * @return std::vector<geometry_msgs::PoseStamped> 
     */
    std::vector<geometry_msgs::PoseStamped> toNavPath(size_t n_steps)
    {
      std::vector<geometry_msgs::PoseStamped> path;
      geometry_msgs::PoseStamped pose;

      double dt_eval = 2 * M_PI / (omega_ * cc::Scalar(n_steps));

      for (size_t i = 0; i < n_steps; ++i)
      {
        cc::Scalar time = i * dt_eval;
        pose.pose = evaluate(time).pos();
        path.push_back(pose);
      }
      return path;
    }

    std::vector<geometry_msgs::Point> toPoints(size_t n_steps)
    {
      std::vector<geometry_msgs::Point> path;

      geometry_msgs::Point point;
      double dt_eval = 2 * M_PI / (omega_ * cc::Scalar(n_steps));

      for (size_t i = 0; i < n_steps; ++i)
      {
        cc::Scalar time = i * dt_eval;
        point = evaluate(time).pos().linear();
        path.push_back(point);
      }
      return path;
    }

  protected:
    cc::Scalar omega_;
    cc::LinearPosition x_i_w_;
  };

  class ParametricCartesianTrajectory : public CartesianTrajectory
  {
  public:
    typedef CartesianTrajectory Base;

  public:
    ParametricCartesianTrajectory(
      cc::Scalar period,
      const cc::LinearPosition &x_i_w,
      cc::Scalar r,
      cc::Scalar a,
      cc::Scalar b,
      cc::Scalar c) 
        : Base(period, x_i_w), r_(r), a_(a), b_(a), c_(c), fac_x_(1.0), fac_y_(1.0)
    {
      // normalize size and change period
      cc::LinearPosition min;
      cc::LinearPosition max;
      cc::Scalar l;
      approximateDimensions(500, min, max, l);
      fac_x_ = r_/(max.x() - min.x());
      fac_y_ = r_/(max.y() - min.y());
      l *= fac_x_;
      cc::Scalar v = 0.15;
      omega_ = 2.0 * M_PI / (l / v);
    }

    virtual ~ParametricCartesianTrajectory()
    {
    }

    virtual cc::CartesianState evaluate(cc::Scalar time)
    {
      cc::CartesianState x_state = cc::CartesianState::Zero();

      x_state.pos().linear().x() = fac_x_*(sin(a_*omega_*time) + 2*sin(2*a_*omega_*time));
      x_state.pos().linear().y() = fac_y_*(cos(b_*omega_*time) - 2*cos(2*b_*omega_*time));
      x_state.pos().linear().z() = 2*fac_x_*-sin(3*c_*omega_*time);
      x_state.pos().linear() += x_i_w_;

      return x_state;
    }

    void approximateDimensions(size_t n_steps, cc::LinearPosition& min, cc::LinearPosition& max, cc::Scalar& l)
    {
      min.setConstant(1000);
      max.setConstant(-1000);
      l = 0.0;

      double dt_eval = 2 * M_PI / (omega_ * cc::Scalar(n_steps));
      cc::Vector3 p0, p1, t, c;
      p0 = evaluate(0.0).pos().linear();
      for (size_t i = 1; i < n_steps; ++i)
      {
        cc::Scalar time = i * dt_eval;
        p1 = evaluate(time).pos().linear();

        l += (p1 - p0).norm();
        p0 = p1;

        min.x() = std::min(min.x(), p1.x());
        min.y() = std::min(min.y(), p1.y());
        min.z() = std::min(min.z(), p1.z());
        max.x() = std::max(max.x(), p1.x());
        max.y() = std::max(max.y(), p1.y());
        max.z() = std::max(max.z(), p1.z());
      }
    }

  protected:
    cc::Scalar r_;
    cc::Scalar a_;
    cc::Scalar b_;
    cc::Scalar c_;
    cc::Scalar fac_x_, fac_y_;
  };

  class PlanarCartesianTrajectory : public CartesianTrajectory
  {
  public:
    typedef CartesianTrajectory Base;

  public:
    /**
   * @brief Construct a new Planar Cartesian Trajectory object
   * 
   * @param period of the circular motion 
   * @param x_i_w inital starting point of the motion
   * @param n_w axis perpendicular to the plane of the circle
   * @param d_w point on that axis to define the center of the circle in space
   */
    PlanarCartesianTrajectory(
        cc::Scalar period,
        const cc::LinearPosition &x_i_w,
        const cc::LinearPosition &n_w,
        const cc::LinearPosition &d_w,
        const cc::Scalar r_factor = 1.0) : Base(period, x_i_w),
                                           n_w_(n_w),
                                           d_w_(d_w),
                                           r_factor_(r_factor)
    {
      defineRotation();
    }

    virtual ~PlanarCartesianTrajectory()
    {
    }

    /**
     * @brief evaluate the trajectory at given time
     * 
     * @param time 
     * @return cc::LinearState 
     */
    cc::CartesianState evaluate(cc::Scalar time)
    {
      cc::Vector3 p, t, c;
      evaluate2d(time, p, t, c);

      // pos, velocity, acceleration in the world frame
      cc::CartesianState x_state = CartesianState::Zero();
      x_state.pos().linear() = R_c_w_ * p + c_w_;
      x_state.vel().linear() = R_c_w_ * t;
      x_state.acc().linear() = R_c_w_ * c;
      x_state.pos().angular() = R_c_w_;

      return x_state;
    }

    virtual void evaluate2d(cc::Scalar time, cc::Vector3& p, cc::Vector3& t, cc::Vector3& c) = 0; 

  private:
    void defineRotation()
    {
      // compute center point c_w such that c_w and x_i_w lie in the same
      // plane perpendicular to axis a_w
      cc::Vector3 a_w = n_w_.normalized();
      c_w_ = d_w_ + (x_i_w_ - d_w_).dot(a_w) * a_w;

      // define a circle frame at center c_w such that its x-axis points into
      // the direction of x_i_w and the z-axis point along the axis a_w.
      r_ = (x_i_w_ - c_w_).norm();
      cc::Vector3 x_axis = (x_i_w_ - c_w_) / r_;
      cc::Vector3 z_axis = a_w;
      cc::Vector3 y_axis = z_axis.cross(x_axis);

      // make a rotation of the circle frame to world frame
      R_c_w_ << x_axis, y_axis, z_axis;
    }

  public:
    cc::Scalar r_;
    cc::Scalar r_factor_;
    cc::LinearPosition d_w_;
    cc::Vector3 n_w_;
    cc::LinearPosition c_w_;
    cc::Rotation3 R_c_w_;
  };

  class CircularCartesianTrajectory : public PlanarCartesianTrajectory
  {
  public:
    typedef PlanarCartesianTrajectory Base;

  public:
    /**
   * @brief Construct a new Circlular Cartesian Trajectory object
   * 
   * @param period of the circular motion 
   * @param x_i_w inital starting point of the motion
   * @param n_w axis perpendicular to the plane of the circle
   * @param d_w point on that axis to define the center of the circle in space
   */
    CircularCartesianTrajectory(
        cc::Scalar period,
        const cc::LinearPosition &x_i_w,
        const cc::LinearPosition &n_w,
        const cc::LinearPosition &d_w,
        const cc::Scalar r_factor = 1.0) : Base(period, x_i_w, n_w, d_w, r_factor)
    {
    }

    virtual ~CircularCartesianTrajectory()
    {
    }

    /**
     * @brief evaluate the trajectory at given time
     * 
     * @param time 
     * @return cc::LinearState 
     */
    virtual void evaluate2d(cc::Scalar time, cc::Vector3& p, cc::Vector3& t, cc::Vector3& c)
    {
      // phase
      cc::Scalar s = omega_ * time;

      // postion, tangential, curvature vector in the circle frame
      p << r_ * cos(s), r_factor_ * r_ * sin(s), 0;
      t << -omega_ * r_ * sin(s), omega_ * r_factor_ * r_ * cos(s), 0;
      c << -omega_ * omega_ * r_ * cos(s), -omega_ * omega_ * r_factor_ * r_ * sin(s), 0;
    }
  };

  class ParametricPlanarCartesianTrajectory : public PlanarCartesianTrajectory
  {
  public:
    typedef PlanarCartesianTrajectory Base;

  public:
    /**
   * @brief Construct a new Circlular Cartesian Trajectory object
   * 
   * @param period of the circular motion 
   * @param x_i_w inital starting point of the motion
   * @param n_w axis perpendicular to the plane of the circle
   * @param d_w point on that axis to define the center of the circle in space
   */
    ParametricPlanarCartesianTrajectory(
        cc::Scalar period,
        const cc::LinearPosition &x_i_w,
        const cc::LinearPosition &n_w,
        const cc::LinearPosition &d_w,
        const cc::Scalar a,
        const cc::Scalar b,
        const cc::Scalar c) : 
          Base(period, x_i_w, n_w, d_w), 
          a_(a), b_(b), c_(c), fac_x_(1.0), fac_y_(1.0)
    {
      // normalize size and change period
      cc::LinearPosition min;
      cc::LinearPosition max;
      cc::Scalar l;
      approximateDimensions(500, min, max, l);
      fac_x_ = r_/(max.x() - min.x());
      fac_y_ = r_/(max.y() - min.y());
      l *= fac_x_;
      cc::Scalar v = 0.15;
      omega_ = 2.0 * M_PI / (l / v);
    }

    virtual ~ParametricPlanarCartesianTrajectory()
    {
    }

    /**
     * @brief evaluate the trajectory at given time
     * 
     * @param time 
     * @return cc::LinearState 
     */
    virtual void evaluate2d(cc::Scalar time, cc::Vector3& p, cc::Vector3& t, cc::Vector3& c)
    {
      // phase
      cc::Scalar s = -omega_ * time;

      // postion, tangential, curvature vector in the circle frame
      p << cos(a_*s) + cos(b_*s)/2 + sin(c_*s)/3, sin(a_*s) + sin(b_*s)/2 + cos(c_*s)/3, 0.0;
      t << -a_*sin(a_*s) - b_*sin(b_*s)/2 + c_*cos(c_*s)/3, -a_*cos(a_*s) + b_*cos(b_*s)/2 - c_*sin(c_*s)/3, 0.0;
      p << -a_*a_*cos(a_*s) - b_*b_*cos(b_*s)/2 - c_*c_*sin(c_*s)/3, -a_*a_*sin(a_*s) - b_*b_*sin(b_*s)/2 - c_*c_*cos(c_*s)/3, 0.0;

      p.x() *= fac_x_;
      p.y() *= fac_y_;

      t.x() *= omega_ * fac_x_;
      t.y() *= omega_ * fac_y_;

      c.x() *= omega_ * omega_ * fac_x_;
      c.y() *= omega_ * omega_ * fac_y_;
    }

    void approximateDimensions(size_t n_steps, cc::LinearPosition& min, cc::LinearPosition& max, cc::Scalar& l)
    {
      min.setConstant(1000);
      max.setConstant(-1000);
      l = 0.0;

      double dt_eval = 2 * M_PI / (omega_ * cc::Scalar(n_steps));
      cc::Vector3 p0, p1, t, c;
      evaluate2d(0.0, p0, t, c);
      for (size_t i = 1; i < n_steps; ++i)
      {
        cc::Scalar time = i * dt_eval;
        evaluate2d(time, p1, t, c);

        l += (p1 - p0).norm();
        p0 = p1;

        min.x() = std::min(min.x(), p1.x());
        min.y() = std::min(min.y(), p1.y());
        max.x() = std::max(max.x(), p1.x());
        max.y() = std::max(max.y(), p1.y());
      }
    }

  protected:
    cc::Scalar fac_x_, fac_y_;
    cc::Scalar a_, b_, c_;
  };

} // namespace ow

#endif
