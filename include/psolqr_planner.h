/**
 * *********************************************************
 *
 * @file: psolqr_planner.h
 * @brief: Contains PSO_LQR local planner class
 * @author: Jing Zongxin
 * @date: 2024-04-20
 * @version: 1.0
 *
 * Copyright (c) 2024, Jing Zongxin
 * All rights reserved.
 *
 * 
 * Note:
 * 
 * In the design of our PSOLQR local path planner, we adopted a decoupled architecture 
 * for planning and control. The core processes of the planning and control algorithms 
 * are implemented separately in pso_optimizer.cpp and lqr_controller.cpp, respectively. 
 * These components offer reserved interfaces for quick and easy access to planning and 
 * control functions. This modular approach allows for convenient changes to planning or 
 * control algorithms without needing to redesign the local planner, offering excellent 
 * scalability.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */

#ifndef PSOLQR_PLANNER_H
#define PSOLQR_PLANNER_H

#include <geometry_msgs/PointStamped.h>
#include <tf2/utils.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <vector>
#include <utility> 
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>

#include "pso_optimizer.h"
#include "lqr_controller.h"


namespace psolqr_planner
{
/**
 * @brief A class implementing a local planner using the PSO and LQR
 */
class PsoLqrPlanner : public nav_core::BaseLocalPlanner
{
public:
  /**
   * @brief Construct a new PsoLqr planner object
   */
  PsoLqrPlanner();

  /**
   * @brief Construct a new PsoLqr planner object
   */
  PsoLqrPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the PsoLqr planner object
   */
  ~PsoLqrPlanner();

  /**
   * @brief Initialization of the local planner
   * @param name        the name to give this instance of the trajectory planner
   * @param tf          a pointer to a transform listener
   * @param costmap_ros the cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Transforms a global plan into a local coordinate frame.
   * @param tf_buffer       The buffer containing transformation frames, used to perform transformations.
   * @param global_plan     The global path to be transformed, consisting of a series of poses each stamped in time.
   * @param robot_pose_odom The current pose of the robot.
   * @param local_frame     The target local frame in which the global plan should be transformed.
   * @param local_plan      A reference to a vector where the transformed local plan will be stored.
   * @return                Returns true if the transformation was successful, false if there were errors such as an empty global plan or failed transformation lookup.
   *
   * This function attempts to transform each pose in the global plan from its original frame into the specified local frame. 
   * It handles transformation errors and ensures that the output vector, local_plan, is populated with the transformed poses 
   * only if all transformations are successful. It uses ROS's tf2_ros library for managing transformations.
   */
  bool transformGlobalPlanToLocal( const tf2_ros::Buffer& tf_buffer, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                                const geometry_msgs::PoseStamped robot_pose_odom, const std::string& local_frame, std::vector<geometry_msgs::PoseStamped>& local_plan);
   


  /**
   * @brief Calculate the distance between two points
   * @param pose1     First point
   * @param pose2     Second point
   * @return the distance between two points
   */
  double calculateDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);


  /**
   * @brief Prune global plan such that already passed poses are cut off, and limit the plan to a specified length and distance from the current pose.
   *
   * The pose of the robot is transformed into the frame of the global plan using the most recent tf transform. If no valid transformation can be found, 
   * the method returns \c false.
   * Initially, the global plan is pruned to remove all poses that are behind the robot beyond a specified distance threshold (\c dist_behind_robot). 
   * Following this, the method further prunes the plan to ensure that the remaining path does not exceed a specified maximum length (\c max_path_length) 
   * and that each pose is within a maximum distance (\c dist_threshold) from the current pose of the robot.
   * If no pose within the specified threshold \c dist_behind_robot is found, nothing will be pruned and the method returns \c false.
   * @remarks Do not choose \c dist_behind_robot too small (not smaller than the cell size of the map), otherwise, no pruning will occur. 
   *          Also, ensure \c max_path_length and \c dist_threshold are set to sensible limits to prevent cutting off necessary parts of the plan.
   * @param tf A reference to a tf buffer
   * @param global_pose The global pose of the robot
   * @param[in,out] global_plan The plan to be transformed and pruned
   * @param dist_behind_robot Distance behind the robot that should be kept [meters]
   * @param max_path_length The maximum allowed length of the remaining path after pruning [meters]
   * @param dist_threshold The maximum distance a pose can be from the current robot's pose to remain in the plan [meters]
   * @return \c true if the plan is pruned, \c false in case of a transform exception or if no pose can be found within the thresholds
   */
  bool pruneGlobalPath(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose,
                                    std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot,
                                    double max_path_length, double dist_threshold); 

  
  /**
   * @brief Convert a ROS path from geometry_msgs::PoseStamped format to a simpler std::vector<std::pair<double, double>> format by reference.
   *
   * This function takes a vector of geometry_msgs::PoseStamped, which typically contains positions of poses in a plan with both position and orientation details, and converts it to a vector of std::pair<double, double>. The conversion results are stored directly into the provided reference to an output vector, eliminating the need for return values and reducing memory overhead. This method is particularly useful for large datasets.
   *
   * @param pose_stamped_vector The input vector containing the poses in geometry_msgs::PoseStamped format, which include position and orientation in a specific coordinate frame.
   * @param[out] path A reference to an output vector where the converted pairs of x and y coordinates will be stored.
 */
  void convertPlan(const std::vector<geometry_msgs::PoseStamped>& pose_stamped_vector, std::vector<std::pair<double, double>>& path);

  /**
   * @brief Check if there is a collision.
   * @param x coordinate (cartesian system)
   * @param y coordinate (cartesian system)
   * @return True is the point collides and false otherwise
  */
  bool collision(double x, double y); 


  /**
   * @brief Checks if the line segment between two points is free from obstacles.
   * @param p1   Pair representing the coordinates (x, y) of the first point.
   * @param p2   Pair representing the coordinates (x, y) of the second point.
   * @return bool True if the line segment is free from obstacles, false otherwise.
   */
  bool isLineFree(const std::pair<double, double> p1,const std::pair<double, double> p2);



  /**
   * @brief Extracts critical path points by optimizing the path plan.
   * @param plan Planned path
   * @param MinPoints The minimum number of extracted critical points
   * @details This function iteratively checks segments of the path for obstacles using isLineFree. If a segment
   *          is free from obstacles, intermediate points are removed to streamline the path. 
   */
  void cutPathPoint(std::vector< std::pair<double, double> >& plan, int MinPoints);

  /**
   * @brief Generates a plan from a given path.
   * @param path    Vector of pairs representing the path in (x, y) coordinates.
   * @param plan    Vector of PoseStamped messages representing the generated plan.
   * @return bool   True if the plan is successfully generated, false otherwise.
   */
  bool getPlanFromPath(std::vector< std::pair<int, int> >& path, std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Tranform from in_pose to out_pose with out frame using tf
   */
  void transformPose(tf2_ros::Buffer* tf, const std::string out_frame, const geometry_msgs::PoseStamped& in_pose,
                     geometry_msgs::PoseStamped& out_pose) const;

  /**
   * @brief Get the Euler Angles from PoseStamped
   * @param ps  PoseStamped to calculate
   * @return  roll, pitch and yaw in XYZ order
   */
  Eigen::Vector3d getEulerAngles(geometry_msgs::PoseStamped& ps);


  /**
   * @brief Optimizes the orientation of poses in a given plan.
   * @param plan   Vector of PoseStamped messages representing the plan to be optimized.
   */
  void optimizationOrientation(std::vector<geometry_msgs::PoseStamped> &plan);

  /**
   * @brief Converts world coordinates into local map grid coordinates.
   * @param wx       The world x-coordinate (double) that needs to be converted.
   * @param wy       The world y-coordinate (double) that needs to be converted.
   * @param mx       Reference to an unsigned int to store the converted x-coordinate in map grid.
   * @param my       Reference to an unsigned int to store the converted y-coordinate in map grid.
   * @return         Returns true if the world coordinates are within the bounds of the local map and conversion is successful; otherwise, returns false.
   *
   * This function converts world coordinates (wx, wy) into corresponding local map grid coordinates (mx, my) 
   * based on the origin and resolution of the map. It checks whether the world coordinates are within the 
   * valid range of the map before performing the conversion. If the converted map coordinates fall outside 
   * the valid range of the map grid dimensions (nx_, ny_), the function returns false.
   */
  bool worldTolocalMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;

  /**
   * @brief Converts  local map grid coordinates into world coordinates 
   * @param wx       The world x-coordinate (double) that needs to be converted.
   * @param wy       The world y-coordinate (double) that needs to be converted.
   * @param mx       Reference to an unsigned int to store the converted x-coordinate in map grid.
   * @param my       Reference to an unsigned int to store the converted y-coordinate in map grid.
   *
   */
  void localmapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;


  /**
   * @brief Set the plan that the controller is following
   * @param orig_global_plan the plan to pass to the controller
   * @return true if the plan was updated successfully, else false
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief Check if the goal pose has been achieved
   * @return true if achieved, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
   * @param cmd_vel will be filled with the velocity command to be passed to the robot base
   * @return true if a valid trajectory was found, else false
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);



private:
  bool initialized_;     // initialized flag
  bool goal_reached_;    // goal reached flag
  tf2_ros::Buffer* tf_;  // transform buffer

  double d_t_;         // control time interval
  Eigen::Matrix3d Q_;  // state error matrix
  Eigen::Matrix2d R_;  // control error matrix


  ros::Publisher plan_pub_, target_pt_pub_, current_pose_pub_;

  // goal parameters
  double goal_x_, goal_y_;
  Eigen::Vector3d goal_rpy_;

protected:

  costmap_2d::Costmap2D* costmap_;                 // costmap
  costmap_2d::Costmap2DROS* costmap_ros_;          // costmap ros
  psolqr_planner::PSO* pso_planner_;               // planner
  psolqr_planner::lqr_controller* lqr_controller_; // controller
  unsigned int nx_, ny_;                           // costmap size
  double origin_x_, origin_y_;                     // costmap origin
  double resolution_;                              // costmap resolution
  std::string frame_id_;
  std::string base_frame_, map_frame_;


  std::vector<geometry_msgs::PoseStamped> global_plan_;  // global plan (map)
  std::vector<geometry_msgs::PoseStamped> global_plan_b; // global plan (base link)
  std::vector<std::pair<double, double>> global_path_;   // global path
  std::vector<std::pair<int, int>> global_path_map_;     // global path
  std::vector<std::pair<int, int>> local_path_;          // local path
  std::vector<geometry_msgs::PoseStamped> local_plan_;   // local plan
  geometry_msgs::PoseStamped global_target_;             // Global Target Point
  geometry_msgs::PointStamped lookahead_pt_;             // lookahead  point
  
  double global_dist_behind_robot_;      // Distance behind the robot that should be kept [meters]
  double global_max_path_length_;        // The maximum allowed length of the remaining path after pruning [meters]
  double global_dist_threshold_;         // The maximum distance a pose can be from the current robot's pose to remain in the plan [meters]
  int Min_num_points_;                   // Minimum number of control points (minimum number of key points per particle)

};
}  // namespace psolqr_planner
#endif