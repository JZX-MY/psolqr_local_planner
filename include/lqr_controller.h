/**
 * *********************************************************
 *
 * @file: lqr_controller.h
 * @brief: Contains the abstract lqr controller class
 * @author: Jing Zongxin
 * @date: 2024-04-20
 * @version: 1.0
 *
 * Copyright (c) 2024, Jing Zongxin.
 * All rights reserved.
 * 
 * Acknowledgments:
 * 
 * The implementation section of the LQR controller in this document references 
 * the lqr_planner from the open-source project ros_motion_planning.
 * Original project author: Yang Haodong
 * Original project link: https://github.com/ai-winter/ros_motion_planning
 * We would like to express our gratitude to Mr. Yang Haodong for his contributions and spirit of sharing, 
 * which have allowed us to further develop and improve upon his work.
 * 
 * Note:
 * 
 * In our work, we have redesigned the framework structure to establish an independent LQR control library, 
 * which is no longer dependent on the local_planner package. It can now easily calculate control commands 
 * through the lqr_control() function, enabling straightforward invocation of this control library.
 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef LQR_CONTROLLER
#define LQR_CONTROLLER

#define LETHAL_COST 253      // lethal cost

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <Eigen/Dense>

namespace psolqr_planner
{
class lqr_controller
{
public:
  /**
   * @brief Construct a new Local Planner object
   */
  lqr_controller(double goal_dist_tol,double rotate_tol, double max_v, double min_v, double max_v_inc, 
                 double max_w, double min_w, double max_w_inc, Eigen::Matrix3d Q, Eigen::Matrix2d R,
                 int lqr_max_iter,double lqr_eps_iter, double lookahead_time, double min_lookahead_dist,
                 double max_lookahead_dist);

  /**
   * @brief Destroy the Local Planner object
   */
  ~lqr_controller();

  /**
   * @brief Regularize angle to [-pi, pi]
   * @param angle the angle (rad) to regularize
   * @return reg_angle the regulated angle
   */
  double regularizeAngle(double angle);


  /**
   * @brief Whether to reach the target pose through rotation operation
   * @param cur   current pose of robot
   * @param goal  goal pose of robot
   * @return true if robot should perform rotation
   */
  bool shouldRotateToGoal(const geometry_msgs::PoseStamped& cur, const geometry_msgs::PoseStamped& goal);

  /**
   * @brief Whether to correct the tracking path with rotation operation
   * @param angle_to_path  the angle deviation
   * @return true if robot should perform rotation
   */
  bool shouldRotateToPath(double angle_to_path, double tolerance = 0.0);

  /**
   * @brief linear velocity regularization
   * @param base_odometry odometry of the robot, to get velocity
   * @param v_d           desired velocity magnitude
   * @return v            regulated linear velocity
   */
  double linearRegularization(nav_msgs::Odometry& base_odometry, double v_d);

  /**
   * @brief angular velocity regularization
   * @param base_odometry odometry of the robot, to get velocity
   * @param w_d           desired angular velocity
   * @return  w           regulated angular velocity
   */
  double angularRegularization(nav_msgs::Odometry& base_odometry, double w_d);

  /**
   * @brief Calculate the look-ahead distance with current speed dynamically
   * @param vt the current speed
   * @return L the look-ahead distance
   */
  double getLookAheadDistance(double vt);

  /**
   * @brief find the point on the path that is exactly the lookahead distance away from the robot
   * @param lookahead_dist    the lookahead distance
   * @param robot_pose_global the robot's pose  [global]
   * @param local_plan        the pruned plan
   * @param pt                the lookahead point
   * @param theta             the angle on traj
   * @param kappa             the curvature on traj
   */
  bool getLookAheadPoint(double lookahead_dist, geometry_msgs::PoseStamped robot_pose_global,
                         const std::vector<geometry_msgs::PoseStamped>& local_plan, geometry_msgs::PointStamped& pt,
                         double& theta, double& kappa);

  /**
   * @brief Calculate distance between the 2 nodes.
   * @param n1 Node 1
   * @param n2 Node 2
   * @return distance between nodes
   */
  double dist(const std::pair<double, double>& node1, const std::pair<double, double>& node2);
  double dist(const geometry_msgs::PoseStamped& node1, const geometry_msgs::PoseStamped& node2);

  /**
   * @brief Clamps a value within a specified range.
   * @tparam T             The type of the values to be clamped.
   * @param value          The value to be clamped.
   * @param low            The lower bound of the range.
   * @param high           The upper bound of the range.
   * @return const T&      The clamped value within the specified range.
   */
   template <typename T>
   const T& clamp(const T& value, const T& low, const T& high) {return std::max(low, std::min(value, high));}

   /**
   * @brief Formula for intersection of a line with a circle centered at the origin
   * @note  https://mathworld.wolfram.com/Circle-LineIntersection.html
   * @param p1/p2     the two point in the segment
   * @param r         the radius of circle centered at the origin
   * @return points   the intersection points of a line and the circle
   */
  std::vector<std::pair<double, double>> circleSegmentIntersection(const std::pair<double, double>& p1,
                                                                  const std::pair<double, double>& p2, double r);

  /**
   * @brief Execute LQR control process
   * @param s   current state
   * @param s_d desired state
   * @param u_r refered control
   * @param d_time duration
   * @return u  control vector
   */
  Eigen::Vector2d _lqrControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r, double d_time);


  /**
   * @brief Performs linear quadratic regulator (LQR) control for robot navigation.
   * @param robot_pose_map   Pose of the robot in the map coordinate system.
   * @param local_plan       Sequence of poses representing the local planned path.
   * @param global_target    Global target pose that the robot is aiming to reach.
   * @param goal_rpy         Vector containing roll, pitch, and yaw goals.
   * @param goal_reached     Reference to a boolean indicating whether the goal has been reached.
   * @param d_time           Duration of time for the control step.
   * @param cmd_vel          Reference to the command velocity to be updated based on the control law.
   * @param lookahead_pt     Point on the path used for the lookahead calculation.
   * @return bool True if the control commands are within normal bounds and the function completes successfully, false if an anomaly in the control instructions is detected.
   */
  bool lqr_control(geometry_msgs::PoseStamped robot_pose_map, std::vector<geometry_msgs::PoseStamped> local_plan, 
                   geometry_msgs::PoseStamped global_target, Eigen::Vector3d goal_rpy, bool & goal_reached, 
                   double d_time, geometry_msgs::Twist& cmd_vel, geometry_msgs::PointStamped & lookahead_pt);


protected:

  double max_v_, min_v_, max_v_inc_;  // linear velocity
  double max_w_, min_w_, max_w_inc_;  // angular velocity

  Eigen::Matrix3d Q_;  // state error matrix
  Eigen::Matrix2d R_;  // control error matrix


  int lqr_max_iter_;   // maximum iteration for ricatti solution
  double lqr_eps_iter_;    // iteration ending threshold

  // if the distance is less than the tolerance value, it is considered to have reached the target position
  double goal_dist_tol_;

  // if the angle deviation is greater than this threshold, perform rotation first
  double rotate_tol_;

  // frame name of odometry
  std::string odom_frame_;

  // odometry helper
  base_local_planner::OdometryHelperRos* odom_helper_;

  costmap_2d::Costmap2DROS* costmap_ros_;  // costmap(ROS wrapper)

  double lookahead_time_;      // lookahead time gain
  double min_lookahead_dist_;  // minimum lookahead distance
  double max_lookahead_dist_;  // maximum lookahead distance
};
}  // namespace local_planner

#endif