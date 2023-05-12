#pragma once

#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_traphalf.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_time_parameterization/trajectory_container.h>

/** @details Adapted from http://docs.ros.org/en/indigo/api/eigen_conversions/html/eigen__kdl_8cpp_source.html#l00090 */
template <typename T>
KDL::Frame toKDL(const T& e)
{
  KDL::Frame k;
  for (unsigned int i = 0; i < 3; ++i)
    k.p[i] = e(i, 3);
  for (unsigned int i = 0; i < 9; ++i)
    k.M.data[i] = e(i / 3, i % 3);

  return k;
}

Eigen::VectorXd fromKDL(const KDL::Twist& k)
{
  Eigen::VectorXd out(6);
  for (int i = 0; i < 6; ++i)
    out[i] = k[i];
  return out;
}

namespace snp_motion_planning
{
class CartesianTimeParameterization
{
public:
  CartesianTimeParameterization(tesseract_kinematics::JointGroup::ConstPtr motion_group, const std::string& tcp,
                                const double max_translational_vel, const double max_translational_acc)
    : motion_group(motion_group)
    , tcp(tcp)
    , max_translational_vel(max_translational_vel)
    , max_translational_acc(max_translational_acc)
  {
  }

  CartesianTimeParameterization(const CartesianTimeParameterization&) = default;
  CartesianTimeParameterization& operator=(const CartesianTimeParameterization&) = default;
  CartesianTimeParameterization(CartesianTimeParameterization&&) = default;
  CartesianTimeParameterization& operator=(CartesianTimeParameterization&&) = default;

  bool compute(tesseract_planning::TrajectoryContainer& trajectory, double max_velocity_scaling_factor = 1.0,
               double max_acceleration_scaling_factor = 1.0) const
  {
    const double eq_radius = 1.0;  // max_translational_velocity / max_rotational_velocity;

    // Set the first point to zero velocity
    trajectory.setData(0, Eigen::VectorXd::Zero(trajectory.dof()), Eigen::VectorXd::Zero(trajectory.dof()), 0.0);

    double duration = 0.0;
    for (Eigen::Index i = 1; i < trajectory.size() - 1; ++i)
    {
      const Eigen::VectorXd& start_joints = trajectory.getPosition(i - 1);
      const Eigen::VectorXd& end_joints = trajectory.getPosition(i);

      // Perform FK to get Cartesian poses
      const KDL::Frame start = toKDL(motion_group->calcFwdKin(start_joints).at(tcp));
      const KDL::Frame end = toKDL(motion_group->calcFwdKin(end_joints).at(tcp));

      // Convert to KDL::Path
      KDL::RotationalInterpolation* rot_interp = new KDL::RotationalInterpolation_SingleAxis();
      KDL::Path* path = new KDL::Path_Line(start, end, rot_interp, eq_radius);
      const double path_length = path->PathLength();

      // Check the path length for validity
      if (path_length < std::numeric_limits<double>::epsilon())  // avoid division by zero
      {
        CONSOLE_BRIDGE_logError("Zero path length");
        return false;
      }

      KDL::VelocityProfile* prof;
      if (i == 1)
      {
        // Half trapezoid velocity profile (front)
        prof = new KDL::VelocityProfile_TrapHalf(max_velocity_scaling_factor * max_translational_vel,
                                                 max_acceleration_scaling_factor * max_translational_acc, true);
      }
      else if (i == trajectory.size() - 2)
      {
        // Half trapezoid velocity profile (back)
        prof = new KDL::VelocityProfile_TrapHalf(max_velocity_scaling_factor * max_translational_vel,
                                                 max_acceleration_scaling_factor * max_translational_acc, false);
      }
      else
      {
        // Rectangular velocity profile
        prof = new KDL::VelocityProfile_Rectangular(max_velocity_scaling_factor * max_translational_vel);
      }

      prof->SetProfile(0.0, path_length);
      const double dt = prof->Duration();

      // Compute the joint velocity and acceleration
      KDL::Trajectory_Segment traj(path, prof);
      KDL::Twist start_vel = traj.Vel(0.0);
      KDL::Twist start_acc = traj.Acc(0.0);
      KDL::Twist end_vel = traj.Vel(dt);
      KDL::Twist end_acc = traj.Acc(dt);

      const Eigen::VectorXd joint_vel = computeJointVelocity(fromKDL(traj.Vel(dt)), end_joints);
      const Eigen::VectorXd joint_acc = computeJointAcceleration(fromKDL(traj.Acc(dt)), end_joints);

      //      const Eigen::VectorXd joint_vel = (end_joints - start_joints) / dt;
      //      const Eigen::VectorXd joint_acc = (joint_vel - trajectory.getVelocity(i - 1)) / dt;

      // Check for joint velocity/acceleration limit violations
      Eigen::Array<bool, Eigen::Dynamic, 1> vel_limit_violations =
          motion_group->getLimits().velocity_limits.array() < joint_vel.array().abs();
      if (vel_limit_violations.any())
      {
        std::stringstream ss;
        ss << "Joint velocity limit violations: "
           << vel_limit_violations.cast<int>().transpose().format(Eigen::IOFormat(1, 0, " ", "\n", "[", "]"));
        CONSOLE_BRIDGE_logError(ss.str().c_str());
        return false;
      }

      Eigen::Array<bool, Eigen::Dynamic, 1> acc_limit_violations =
          motion_group->getLimits().acceleration_limits.array() < joint_acc.array().abs();
      if (acc_limit_violations.any())
      {
        std::stringstream ss;
        ss << "Joint acceleration limit violations: "
           << acc_limit_violations.cast<int>().transpose().format(Eigen::IOFormat(1, 0, " ", "\n", "[", "]"));
        CONSOLE_BRIDGE_logError(ss.str().c_str());
        return false;
      }

      // Increment the duration
      duration += dt;

      // Update the trajectory
      trajectory.setData(i, joint_vel, joint_acc, duration);
    }

    // Set the last point to zero velocity/acceleration
    trajectory.setData(trajectory.size(), Eigen::VectorXd::Zero(trajectory.dof()),
                       Eigen::VectorXd::Zero(trajectory.dof()), duration);

    return true;
  }

  //  bool compute(tesseract_planning::TrajectoryContainer& trajectory, const std::vector<double>& max_velocity,
  //               const std::vector<double>& max_acceleration, double max_velocity_scaling_factor = 1.0,
  //               double max_acceleration_scaling_factor = 1.0) const
  //  {
  //    return false;
  //  }

  //  bool compute(tesseract_planning::TrajectoryContainer& trajectory,
  //               const Eigen::Ref<const Eigen::VectorXd>& max_velocity,
  //               const Eigen::Ref<const Eigen::VectorXd>& max_acceleration, double max_velocity_scaling_factor = 1.0,
  //               double max_acceleration_scaling_factor = 1.0) const
  //  {
  //    return false;
  //  }

  //  bool compute(tesseract_planning::TrajectoryContainer& trajectory,
  //               const Eigen::Ref<const Eigen::VectorXd>& max_velocity,
  //               const Eigen::Ref<const Eigen::VectorXd>& max_acceleration,
  //               const Eigen::Ref<const Eigen::VectorXd>& max_velocity_scaling_factors,
  //               const Eigen::Ref<const Eigen::VectorXd>& max_acceleration_scaling_factors) const
  //  {
  //    return false;
  //  }

private:
  Eigen::VectorXd computeJointVelocity(const Eigen::VectorXd& twist, const Eigen::VectorXd& joint_state) const
  {
    Eigen::MatrixXd jac = motion_group->calcJacobian(joint_state, tcp);
    return jac.colPivHouseholderQr().solve(twist);
  }

  Eigen::VectorXd computeJointAcceleration(const Eigen::VectorXd& twist, const Eigen::VectorXd& joint_state) const
  {
    Eigen::MatrixXd jac = motion_group->calcJacobian(joint_state, tcp);
    auto ret = jac.transpose() * twist;
    return ret;
  }

  tesseract_kinematics::JointGroup::ConstPtr motion_group;
  const std::string tcp;
  const double max_translational_vel;
  const double max_translational_acc;
};

}  // namespace snp_motion_planning
