// VNAV_LAB4_PARTS_IMPLEMENTED
//
// Created by stewart on 9/20/20.
//

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstddef>

class WaypointFollower {
  [[maybe_unused]] ros::Subscriber currentStateSub;
  [[maybe_unused]] ros::Subscriber poseArraySub;
  ros::Publisher desiredStatePub;

  // Current state
  Eigen::Vector3d x = Eigen::Vector3d::Zero();
  double current_yaw_ = 0.0;  // VNAV_LAB4_START_YAW_FROM_ODOM
  bool have_state_ = false;
  bool have_pending_ = false;
  geometry_msgs::PoseArray pending_;

  ros::Timer desiredStateTimer;

  ros::Time trajectoryStartTime;
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory yaw_trajectory;

  void onCurrentState(nav_msgs::Odometry const& cur_state) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.1 |  16.485 - Fall 2021  - Lab 4 coding assignment (5 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //
    //  Populate the variable x, which encodes the current world position of the
    //  UAV
    // ~~~~ begin solution
    x = Eigen::Vector3d(
        cur_state.pose.pose.position.x,
        cur_state.pose.pose.position.y,
        cur_state.pose.pose.position.z);

    current_yaw_ = tf::getYaw(cur_state.pose.pose.orientation);
    have_state_ = true;

    if (have_pending_ && trajectory.empty()) {
      have_pending_ = false;
      generateOptimizedTrajectory(pending_);
    }
    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //                                 end part 1.1
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

  void generateOptimizedTrajectory(geometry_msgs::PoseArray const& poseArray) {
    if (poseArray.poses.size() < 1) {
      ROS_ERROR("Must have at least one pose to generate trajectory!");
      trajectory.clear();
      yaw_trajectory.clear();
      return;
    }

    if (!trajectory.empty()) return;

    if (!have_state_) {
      pending_ = poseArray;
      have_pending_ = true;
      ROS_WARN_THROTTLE(2.0, "Waiting for /current_state");
      return;
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.2 |  16.485 - Fall 2021  - Lab 4 coding assignment (35 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //
    //  We are using the mav_trajectory_generation library
    //  (https://github.com/ethz-asl/mav_trajectory_generation) to perform
    //  trajectory optimization given the waypoints (based on the position and
    //  orientation of the gates on the race course).
    //  We will be finding the trajectory for the position and the trajectory
    //  for the yaw in a decoupled manner.
    //  In this section:
    //  1. Fill in the correct number for D, the dimension we should apply to
    //  the solver to find the positional trajectory
    //  2. Correctly populate the Vertex::Vector structure below (vertices,
    //  yaw_vertices) using the position of the waypoints and the yaw of the
    //  waypoints respectively
    //
    //  Hints:
    //  1. Use vertex.addConstraint(POSITION, position) where position is of
    //  type Eigen::Vector3d to enforce a waypoint position.
    //  2. Use vertex.addConstraint(ORIENTATION, yaw) where yaw is a double
    //  to enforce a waypoint yaw.
    //  3. Remember angle wraps around 2 pi. Be careful!
    //  4. For the ending waypoint for position use .makeStartOrEnd as seen with
    //  the starting waypoint instead of .addConstraint as you would do for the
    //  other waypoints.
    //
    // ~~~~ begin solution
    const int D = 3;

    mav_trajectory_generation::Vertex::Vector vertices;
    mav_trajectory_generation::Vertex::Vector yaw_vertices;

    using namespace mav_trajectory_generation::derivative_order;

    mav_trajectory_generation::Vertex start_pos(D);
    mav_trajectory_generation::Vertex start_yaw(1);

    start_pos.makeStartOrEnd(x, SNAP);
    start_yaw.makeStartOrEnd(Eigen::VectorXd::Constant(1, current_yaw_), SNAP);

    vertices.push_back(start_pos);
    yaw_vertices.push_back(start_yaw);

    double last_yaw = current_yaw_;

    for (std::size_t i = 0; i < poseArray.poses.size(); ++i) {
      const auto& pose = poseArray.poses[i];

      const Eigen::Vector3d pos(
          pose.position.x,
          pose.position.y,
          pose.position.z);

      const double yaw = tf::getYaw(pose.orientation);

      last_yaw += std::atan2(
          std::sin(yaw - last_yaw),
          std::cos(yaw - last_yaw));

      const Eigen::VectorXd yaw_vec =
          Eigen::VectorXd::Constant(1, last_yaw);

      const bool final_point =
          (i + 1 == poseArray.poses.size());

      mav_trajectory_generation::Vertex pos_vertex(D);
      mav_trajectory_generation::Vertex yaw_vertex(1);

      if (final_point) {
        pos_vertex.makeStartOrEnd(pos, SNAP);
        yaw_vertex.makeStartOrEnd(yaw_vec, SNAP);
      } else {
        pos_vertex.addConstraint(POSITION, pos);
        yaw_vertex.addConstraint(ORIENTATION, yaw_vec);
      }

      vertices.push_back(pos_vertex);
      yaw_vertices.push_back(yaw_vertex);
    }

    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //                                 end part 1.2
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // ============================================================
    // Estimate the time to complete each segment of the trajectory
    // ============================================================

    // HINT: play with these segment times and see if you can finish
    // the race course faster!
    std::vector<double> segment_times;
    const double v_max = 8.0;
    const double a_max = 4.0;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);

    // VNAV_LAB4_UNIFORM_TIME_SCALE_X4
    // Slow down the entire Lab 4 trajectory while preserving
    // its spatial waypoints and zero-derivative endpoints.
    const double lab4_time_scale = 4.0;
    for (double& segment_time : segment_times) {
      segment_time *= lab4_time_scale;
    }


    // =====================================================
    // Solve for the optimized trajectory (linear optimizer)
    // =====================================================
    // Position
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(D);
    opt.setupFromVertices(vertices, segment_times, SNAP);
    opt.solveLinear();

    // Yaw
    mav_trajectory_generation::PolynomialOptimization<N> yaw_opt(1);
    yaw_opt.setupFromVertices(yaw_vertices, segment_times, SNAP);
    yaw_opt.solveLinear();

    // ============================
    // Get the optimized trajectory
    // ============================
    mav_trajectory_generation::Segment::Vector segments;
    //        opt.getSegments(&segments); // Unnecessary?
    opt.getTrajectory(&trajectory);
    yaw_opt.getTrajectory(&yaw_trajectory);
    trajectoryStartTime = ros::Time::now();

    ROS_INFO("Generated optimizes trajectory from %d waypoints",
             vertices.size());
  }

  void publishDesiredState(ros::TimerEvent const& ev) {
    if (trajectory.empty() || yaw_trajectory.empty()) return;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.3 |  16.485 - Fall 2021  - Lab 4 coding assignment (15 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //
    //  Finally we get to send commands to our controller! First fill in
    //  properly the value for 'nex_point.time_from_start' and 'sampling_time'
    //  (hint: not 0) and after extracting the state information from our
    //  optimized trajectory, finish populating next_point.
    //
    // ~~~~ begin solution
    trajectory_msgs::MultiDOFJointTrajectoryPoint next_point;

    double sec = (ros::Time::now() - trajectoryStartTime).toSec();

    if (sec < 0.0) sec = 0.0;
    if (sec > trajectory.getMaxTime())
      sec = trajectory.getMaxTime();

    next_point.time_from_start = ros::Duration(sec);

    using namespace mav_trajectory_generation::derivative_order;

    const Eigen::Vector3d pos =
        trajectory.evaluate(sec, POSITION);
    const Eigen::Vector3d vel =
        trajectory.evaluate(sec, VELOCITY);
    const Eigen::Vector3d acc =
        trajectory.evaluate(sec, ACCELERATION);

    const double yaw =
        yaw_trajectory.evaluate(sec, ORIENTATION)(0);
    const double yaw_rate =
        yaw_trajectory.evaluate(sec, VELOCITY)(0);
    const double yaw_acc =
        yaw_trajectory.evaluate(sec, ACCELERATION)(0);

    next_point.transforms.resize(1);
    next_point.velocities.resize(1);
    next_point.accelerations.resize(1);

    next_point.transforms[0].translation.x = pos.x();
    next_point.transforms[0].translation.y = pos.y();
    next_point.transforms[0].translation.z = pos.z();

    next_point.transforms[0].rotation =
        tf::createQuaternionMsgFromYaw(yaw);

    next_point.velocities[0].linear.x = vel.x();
    next_point.velocities[0].linear.y = vel.y();
    next_point.velocities[0].linear.z = vel.z();
    next_point.velocities[0].angular.z = yaw_rate;

    next_point.accelerations[0].linear.x = acc.x();
    next_point.accelerations[0].linear.y = acc.y();
    next_point.accelerations[0].linear.z = acc.z();
    next_point.accelerations[0].angular.z = yaw_acc;

    desiredStatePub.publish(next_point);

    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //                                 end part 1.3
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

 public:
  explicit WaypointFollower(ros::NodeHandle& nh) {
    currentStateSub = nh.subscribe(
        "/current_state", 1, &WaypointFollower::onCurrentState, this);
    poseArraySub = nh.subscribe("/desired_traj_vertices",
                                1,
                                &WaypointFollower::generateOptimizedTrajectory,
                                this);
    desiredStatePub =
        nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/desired_state", 1);
    desiredStateTimer = nh.createTimer(
        ros::Rate(5), &WaypointFollower::publishDesiredState, this);
    desiredStateTimer.start();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generation_node");
  ros::NodeHandle nh;

  WaypointFollower waypointFollower(nh);

  ros::spin();
  return 0;
}
