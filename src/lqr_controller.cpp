/**
 * *********************************************************
 *
 * @file: lqr_controller.cpp
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

#include <tf2/utils.h>

#include "lqr_controller.h"

namespace psolqr_planner
{
/**
 * @brief Construct a new Local Planner object
 */
lqr_controller::lqr_controller(double goal_dist_tol,double rotate_tol, double max_v, double min_v, double max_v_inc, 
                               double max_w, double min_w, double max_w_inc, Eigen::Matrix3d Q, Eigen::Matrix2d R,
                               int lqr_max_iter,double lqr_eps_iter, double lookahead_time, double min_lookahead_dist,
                               double max_lookahead_dist)
  : odom_frame_("odom")
  , goal_dist_tol_(goal_dist_tol)
  , rotate_tol_(rotate_tol)
  , max_v_(max_v)
  , min_v_(min_v)
  , max_v_inc_(max_v_inc)
  , max_w_(max_w)
  , min_w_(min_w)
  , max_w_inc_(max_w_inc)
  , Q_(Q)
  , R_(R)
  , lqr_max_iter_(lqr_max_iter)
  , lqr_eps_iter_(lqr_eps_iter)
  , lookahead_time_(lookahead_time)
  , min_lookahead_dist_(min_lookahead_dist)
  , max_lookahead_dist_(max_lookahead_dist)
  , costmap_ros_(nullptr)
{
  odom_helper_ = new base_local_planner::OdometryHelperRos(odom_frame_);
}

/**
 * @brief Destroy the Local Planner object
 */
lqr_controller::~lqr_controller()
{
  delete odom_helper_;
}

/**
 * @brief Regularize angle to [-pi, pi]
 * @param angle the angle (rad) to regularize
 * @return reg_angle the regulated angle
 */
double lqr_controller::regularizeAngle(double angle)
{
  return angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}


/**
 * @brief Whether to reach the target pose through rotation operation
 * @param cur  current pose of robot
 * @param goal goal pose of robot
 * @return true if robot should perform rotation
 */
bool lqr_controller::shouldRotateToGoal(const geometry_msgs::PoseStamped& cur, const geometry_msgs::PoseStamped& goal)
{
  std::pair<double, double> p1(cur.pose.position.x, cur.pose.position.y);
  std::pair<double, double> p2(goal.pose.position.x, goal.pose.position.y);

  return dist(p1, p2) < goal_dist_tol_;
}

/**
 * @brief Whether to correct the tracking path with rotation operation
 * @param angle_to_path the angle deviation
 * @return true if robot should perform rotation
 */
bool lqr_controller::shouldRotateToPath(double angle_to_path, double tolerance)
{
  return (tolerance && (angle_to_path > tolerance)) || (!tolerance && (angle_to_path > rotate_tol_));
}

/**
 * @brief linear velocity regularization
 * @param base_odometry odometry of the robot, to get velocity
 * @param v_d           desired velocity magnitude
 * @return v            regulated linear velocity
 */
double lqr_controller::linearRegularization(nav_msgs::Odometry& base_odometry, double v_d)
{
  double v = std::hypot(base_odometry.twist.twist.linear.x, base_odometry.twist.twist.linear.y);
  double v_inc = v_d - v;

  if (std::fabs(v_inc) > max_v_inc_)
    v_inc = std::copysign(max_v_inc_, v_inc);

  double v_cmd = v + v_inc;
  if (std::fabs(v_cmd) > max_v_)
    v_cmd = std::copysign(max_v_, v_cmd);
  else if (std::fabs(v_cmd) < min_v_)
    v_cmd = std::copysign(min_v_, v_cmd);

  return v_cmd;
}

/**
 * @brief angular velocity regularization
 * @param base_odometry odometry of the robot, to get velocity
 * @param w_d           desired angular velocity
 * @return w            regulated angular velocity
 */
double lqr_controller::angularRegularization(nav_msgs::Odometry& base_odometry, double w_d)
{
  if (std::fabs(w_d) > max_w_)
    w_d = std::copysign(max_w_, w_d);

  double w = base_odometry.twist.twist.angular.z;
  double w_inc = w_d - w;

  if (std::fabs(w_inc) > max_w_inc_)
    w_inc = std::copysign(max_w_inc_, w_inc);

  double w_cmd = w + w_inc;
  if (std::fabs(w_cmd) > max_w_)
    w_cmd = std::copysign(max_w_, w_cmd);
  else if (std::fabs(w_cmd) < min_w_)
    w_cmd = std::copysign(min_w_, w_cmd);

  return w_cmd;
}


// Calculate distance between the 2 nodes.
double lqr_controller::dist(const std::pair<double, double>& node1, const std::pair<double, double>& node2)
{
  return std::hypot(node1.first - node2.first, node1.second - node2.second);
}

double lqr_controller::dist(const geometry_msgs::PoseStamped& node1, const geometry_msgs::PoseStamped& node2)
{
  return std::hypot(node1.pose.position.x - node2.pose.position.x, node1.pose.position.y - node2.pose.position.y);
}

/**
 * @brief Calculate the look-ahead distance with current speed dynamically
 * @param vt the current speed
 * @return L the look-ahead distance
 */
double lqr_controller::getLookAheadDistance(double vt)
{
  double lookahead_dist = fabs(vt) * lookahead_time_;
  return clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
}

/**
 * @brief find the point on the path that is exactly the lookahead distance away from the robot
 * @param lookahead_dist    the lookahead distance
 * @param robot_pose_global the robot's pose  [global]
 * @param local_plan        the pruned plan
 * @param pt                the lookahead point
 * @param theta             the angle on traj
 * @param kappa             the curvature on traj
 */
bool lqr_controller::getLookAheadPoint(double lookahead_dist, geometry_msgs::PoseStamped robot_pose_global,
                                     const std::vector<geometry_msgs::PoseStamped>& local_plan,
                                     geometry_msgs::PointStamped& pt, double& theta, double& kappa)
{
  double rx = robot_pose_global.pose.position.x;
  double ry = robot_pose_global.pose.position.y;

  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(local_plan.begin(), local_plan.end(), [&](const geometry_msgs::PoseStamped& ps) {
    return dist(ps, robot_pose_global) >= lookahead_dist;
  });

  std::vector<geometry_msgs::PoseStamped>::const_iterator prev_pose_it;
  std::vector<geometry_msgs::PoseStamped>::const_iterator pprev_pose_it;
  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == local_plan.end())
  {
    goal_pose_it = std::prev(local_plan.end());
    prev_pose_it = std::prev(goal_pose_it);
    pprev_pose_it = std::prev(prev_pose_it);
    pt.point.x = goal_pose_it->pose.position.x;
    pt.point.y = goal_pose_it->pose.position.y;
    kappa = 0;
  }
  else
  {
    // find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    prev_pose_it = std::prev(goal_pose_it);
    pprev_pose_it = std::prev(prev_pose_it);

    double px = prev_pose_it->pose.position.x;
    double py = prev_pose_it->pose.position.y;
    double gx = goal_pose_it->pose.position.x;
    double gy = goal_pose_it->pose.position.y;

    // transform to the robot frame so that the circle centers at (0,0)
    std::pair<double, double> prev_p(px - rx, py - ry);
    std::pair<double, double> goal_p(gx - rx, gy - ry);
    std::vector<std::pair<double, double>> i_points = circleSegmentIntersection(prev_p, goal_p, lookahead_dist);

    // check i_points 
    if (i_points.empty()) 
    {
      ROS_INFO("i_points is empty");
       return false;
    }

    pt.point.x = i_points[0].first + rx;
    pt.point.y = i_points[0].second + ry;

    double ax = pprev_pose_it->pose.position.x;
    double ay = pprev_pose_it->pose.position.y;
    double bx = prev_pose_it->pose.position.x;
    double by = prev_pose_it->pose.position.y;
    double cx = goal_pose_it->pose.position.x;
    double cy = goal_pose_it->pose.position.y;

    double a = std::hypot(bx - cx, by - cy);
    double b = std::hypot(cx - ax, cy - ay);
    double c = std::hypot(ax - bx, ay - by);

    double cosB = (a * a + c * c - b * b) / (2 * a * c);
    double sinB = std::sin(std::acos(cosB));

    double cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    kappa = std::copysign(2 * sinB / b, cross);
  }

  pt.header.frame_id = goal_pose_it->header.frame_id;
  pt.header.stamp = goal_pose_it->header.stamp;

  theta = atan2(goal_pose_it->pose.position.y - prev_pose_it->pose.position.y,
                goal_pose_it->pose.position.x - prev_pose_it->pose.position.x);

  
  return true;
}

/**
 * @brief Formula for intersection of a line with a circle centered at the origin
 * @note  https://mathworld.wolfram.com/Circle-LineIntersection.html
 * @param p1/p2     the two point in the segment
 * @param r         the radius of circle centered at the origin
 * @return points   the intersection points of a line and the circle
 */
std::vector<std::pair<double, double>> lqr_controller::circleSegmentIntersection(const std::pair<double, double>& p1,
                                                                                 const std::pair<double, double>& p2, double r)
{
  std::vector<std::pair<double, double>> i_points;

  double x1 = p1.first;
  double x2 = p2.first;
  double y1 = p1.second;
  double y2 = p2.second;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // the first element is the point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  double delta = std::sqrt(r * r * dr2 - D * D);

  if (delta >= 0)
  {
    if (delta == 0)
      i_points.emplace_back(D * dy / dr2, -D * dx / dr2);
    else
    {
      i_points.emplace_back((D * dy + std::copysign(1.0, dd) * dx * delta) / dr2,
                            (-D * dx + std::copysign(1.0, dd) * dy * delta) / dr2);
      i_points.emplace_back((D * dy - std::copysign(1.0, dd) * dx * delta) / dr2,
                            (-D * dx - std::copysign(1.0, dd) * dy * delta) / dr2);
    }
  }

  return i_points;
}


// Execute LQR control process
Eigen::Vector2d lqr_controller::_lqrControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r ,double d_time)
{


  Eigen::Vector2d u;
  Eigen::Vector3d e(s - s_d);
  e[2] = regularizeAngle(e[2]);


  // state equation on error
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  A(0, 2) = -u_r[0] * sin(s_d[2]) * d_time;
  A(1, 2) = u_r[0] * cos(s_d[2]) * d_time;



  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 2);
  B(0, 0) = cos(s_d[2]) * d_time;
  B(1, 0) = sin(s_d[2]) * d_time;
  B(2, 1) = d_time;



  // discrete iteration Ricatti equation
  Eigen::Matrix3d P, P_;
  P = Q_;
  for (int i = 0; i < lqr_max_iter_; ++i)
  {
    Eigen::Matrix2d temp = R_ + B.transpose() * P * B;
    P_ = Q_ + A.transpose() * P * A - A.transpose() * P * B * temp.inverse() * B.transpose() * P * A;
    if ((P - P_).array().abs().maxCoeff() < lqr_eps_iter_)
      break;
    P = P_;
  }



  // feedback
  Eigen::MatrixXd K = -(R_ + B.transpose() * P_ * B).inverse() * B.transpose() * P_ * A;

  // std::cout<<"check Q_:"<<Q_<<std::endl;
  // std::cout<<"check R_:"<<R_<<std::endl;
  // std::cout<<"check B:"<<B<<std::endl;
  // std::cout<<"check P_:"<<P_<<std::endl;
  // std::cout<<"check A:"<<A<<std::endl;



  u = u_r + K * e;

  // std::cout<<"check ur:"<<u_r<<std::endl;
  // std::cout<<"check e:"<<e<<std::endl;
  // std::cout<<"check K:"<<K<<std::endl;
  // std::cout<<"check u:"<<u<<std::endl;

  return u;
}



bool lqr_controller::lqr_control(geometry_msgs::PoseStamped robot_pose_map, std::vector<geometry_msgs::PoseStamped> local_plan, 
                                 geometry_msgs::PoseStamped global_target, Eigen::Vector3d goal_rpy, bool & goal_reached, 
                                 double d_time, geometry_msgs::Twist& cmd_vel, geometry_msgs::PointStamped& lookahead_pt)
{
  // odometry observation - getting robot velocities in robot frame
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);
  


  // calculate look-ahead distance
  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double wt = base_odom.twist.twist.angular.z;
  double L = getLookAheadDistance(vt);



  // get the particular point on the path at the lookahead distance
  double theta_trj, kappa;
  bool ObtainAheadPoint = getLookAheadPoint(L, robot_pose_map, local_plan, lookahead_pt, theta_trj, kappa);

  if (!ObtainAheadPoint)
  {
    ROS_INFO("ERROR: Forward looking point acquisition failed");
    return false;
  }




  // current angle
  double theta = tf2::getYaw(robot_pose_map.pose.orientation);  // [-pi, pi]



  // calculate commands
  if (shouldRotateToGoal(robot_pose_map, global_target))
  {
    double e_theta = regularizeAngle(goal_rpy.z() - theta);

    // orientation reached
    if (!shouldRotateToPath(std::fabs(e_theta)))
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached = true;
    }
    // orientation not reached
    else
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angularRegularization(base_odom, e_theta / d_time);
    }


  }
  else
  {
    Eigen::Vector3d s(robot_pose_map.pose.position.x, robot_pose_map.pose.position.y, theta);  // current state
    Eigen::Vector3d s_d(lookahead_pt.point.x, lookahead_pt.point.y, theta_trj);                // desired state
    Eigen::Vector2d u_r(vt, vt * kappa);                                                       // refered input
    Eigen::Vector2d u = _lqrControl(s, s_d, u_r, d_time);

    // std::cout<<"s:"<<s<<std::endl;
    // std::cout<<"s_d:"<<s_d<<std::endl;
    // std::cout<<"u_r:"<<u_r<<std::endl;
    // std::cout<<"u:"<<u<<std::endl;



    cmd_vel.linear.x = linearRegularization(base_odom, u[0]);

    ROS_INFO("linear.x: %f", cmd_vel.linear.x);
    cmd_vel.angular.z = angularRegularization(base_odom, u[1]);
    ROS_INFO("angular.z: %f", cmd_vel.angular.z);

    // Control instruction anomaly detection
    if((std::fabs(cmd_vel.linear.x)> 100000)||(std::fabs(cmd_vel.angular.z)> 100000))
    {
      ROS_INFO("Control command abnormal, stop moving");
      return false;
    }

  }

  return true;

}



}  // namespace local_planner