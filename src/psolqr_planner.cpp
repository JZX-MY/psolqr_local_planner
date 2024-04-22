/**
 * *********************************************************
 *
 * @file: psolqr_planner.cpp
 * @brief: Contains PSO_LQR local planner class
 * @author: Jing Zongxin
 * @date: 2024-04-20
 * @version: 1.0
 *
 * Copyright (c) 2024, Jing Zongxin
 * All rights reserved.
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

#include <pluginlib/class_list_macros.h>
#include "psolqr_planner.h"

PLUGINLIB_EXPORT_CLASS(psolqr_planner::PsoLqrPlanner, nav_core::BaseLocalPlanner)

namespace psolqr_planner
{

// Construct a new psolqr planner object
PsoLqrPlanner::PsoLqrPlanner() : initialized_(false), goal_reached_(false), tf_(nullptr) 
{
}

// Construct a new psolqr planner object
PsoLqrPlanner::PsoLqrPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : PsoLqrPlanner()
{
  initialize(name, tf, costmap_ros);
}

// Destroy the psolqr planner object
PsoLqrPlanner::~PsoLqrPlanner()
{
}


//Initialization of the local planner
void PsoLqrPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    tf_ = tf;

    // Initialize map
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros->getCostmap();
    frame_id_ = costmap_ros->getGlobalFrameID();

    // get costmap properties
    nx_ = costmap_->getSizeInCellsX(), ny_ = costmap_->getSizeInCellsY();
    origin_x_ = costmap_->getOriginX(), origin_y_ = costmap_->getOriginY();
    resolution_ = costmap_->getResolution();

    // ROS Publisher
    ros::NodeHandle nh = ros::NodeHandle("~/" + name);
    plan_pub_ = nh.advertise<nav_msgs::Path>("plan",1); 
    target_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>("/target_point", 10);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    // PSO param
    bool pub_particles;
    int n_particles,n_inherited,pointNum,max_speed,pso_max_iter;
    double obs_factor, w_inertial, w_social, w_cognitive;
    double state_init_ratio, disturbance_state_init_ratio;
    nh.param("n_particles", n_particles, 50);        // number of particles
    nh.param("n_inherited", n_inherited, 20);        // number of inherited particles
    nh.param("pointNum", pointNum, 3);               // number of position points contained in each particle
    nh.param("obs_factor", obs_factor, 0.39);        // obstacle factor(greater means obstacles)
    nh.param("max_speed", max_speed,40);             // The maximum velocity of particle motion
    nh.param("w_inertial", w_inertial,1.0);          // inertia weight
    nh.param("w_social", w_social, 2.0);             // social weight
    nh.param("w_cognitive", w_cognitive, 1.2);       // cognitive weight
    nh.param("pso_state_init_ratio", state_init_ratio, 0.2);                            // Set the generation mode for the initial position points of the particle swarm
    nh.param("pso_disturbance_state_init_ratio", disturbance_state_init_ratio, 0.6);    // Set the generation mode for the initial position points of the particle swarm
    nh.param("pub_particles", pub_particles, false); // Whether to publish particles
    nh.param("pso_max_iter", pso_max_iter, 5);       // maximum iterations

    // global path param
    nh.param("global_dist_behind_robot", global_dist_behind_robot_, 0.2);   // Distance behind the robot that should be kept [meters]
    nh.param("global_max_path_length", global_max_path_length_, 2.0);       // The maximum allowed length of the remaining path after pruning [meters]
    nh.param("global_dist_threshold", global_dist_threshold_, 1.5);         // The maximum distance a pose can be from the current robot's pose to remain in the plan [meters]
    Min_num_points_=pointNum+2;                                               // Minimum number of control poin

    //LQR param
    double goal_dist_tol,rotate_tol,lookahead_time,min_lookahead_dist,max_lookahead_dist;
    double max_v, min_v, max_v_inc, max_w, min_w, max_w_inc; 
    int lqr_max_iter;   
    double lqr_eps_iter;    

    // base
    nh.param("goal_dist_tolerance", goal_dist_tol, 0.2);
    nh.param("rotate_tolerance", rotate_tol, 0.5);
    nh.param("base_frame", base_frame_, std::string("base_link"));
    nh.param("map_frame", map_frame_, std::string("map"));

    // lookahead
    nh.param("lookahead_time", lookahead_time, 1.5);
    nh.param("min_lookahead_dist", min_lookahead_dist, 0.3);
    nh.param("max_lookahead_dist", max_lookahead_dist, 0.9);

    // linear velocity
    nh.param("max_v", max_v, 0.5);
    nh.param("min_v", min_v, 0.0);
    nh.param("max_v_inc", max_v_inc, 0.5);

    // angular velocity
    nh.param("max_w", max_w, 1.57);
    nh.param("min_w", min_w, 0.0);
    nh.param("max_w_inc", max_w_inc, 1.57);

    // iteration for ricatti solution
    nh.param("lqr_max_iter", lqr_max_iter, 100);
    nh.param("lqr_eps_iter", lqr_eps_iter, 1e-4);

    // weight matrix for penalizing state error while tracking [x,y,theta]
    Q_.setZero();
    std::vector<double> diag_vec;
    nh.getParam("Q_matrix_diag", diag_vec);
    for (size_t i = 0; i < diag_vec.size(); ++i)
      Q_(i, i) = diag_vec[i];

    // weight matrix for penalizing input error while tracking[v, w]
    R_.setZero();
    nh.getParam("R_matrix_diag", diag_vec);
    for (size_t i = 0; i < diag_vec.size(); ++i)
      R_(i, i) = diag_vec[i];
     

    double controller_freqency;
    nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
    d_t_ = 1 / controller_freqency;

    // Initialize PSO Planner
    pso_planner_ = new PSO(nx_, ny_, resolution_ ,origin_x_,origin_y_, n_particles,n_inherited, pointNum, w_inertial, w_social, w_cognitive, obs_factor, max_speed ,state_init_ratio, disturbance_state_init_ratio, pub_particles, pso_max_iter);
    ROS_INFO("PSO planner initialized!");
    // Initialize LQR Controller
    lqr_controller_ = new lqr_controller(goal_dist_tol,rotate_tol, max_v, min_v, max_v_inc, max_w, min_w, max_w_inc, Q_, R_,lqr_max_iter,lqr_eps_iter,lookahead_time,min_lookahead_dist,max_lookahead_dist);
    ROS_INFO("LQR controller initialized!");
    
  }
  else
  {
    ROS_WARN("PSOLQR planner has already been initialized.");
  }
}


// Optimizes the orientation of poses in a given plan.
void PsoLqrPlanner::optimizationOrientation(std::vector<geometry_msgs::PoseStamped> &plan)
{
  size_t num = plan.size()-1;
  if(num < 1)
    return;
  for(size_t i=0;i<num;i++)
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, atan2(plan[i+1].pose.position.y - plan[i].pose.position.y, plan[i+1].pose.position.x - plan[i].pose.position.x));
    plan[i].pose.orientation = tf2::toMsg(q);
  }
}



double PsoLqrPlanner::calculateDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) 
{
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;
  return sqrt(dx * dx + dy * dy);
}

// Prune global plan such that already passed poses are cut off, and limit the plan to a specified length and distance from the current pose.
bool PsoLqrPlanner::pruneGlobalPath(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose,
                                    std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot,
                                    double max_path_length, double dist_threshold) 
{
  if (global_plan.empty())
    return true;

  try {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id,
                                                                                   global_pose.header.frame_id, ros::Time(0));
    geometry_msgs::PoseStamped robot;
    tf2::doTransform(global_pose, robot, global_to_plan_transform);

    double dist_behind_robot_sq = dist_behind_robot * dist_behind_robot;
    double dist_threshold_sq = dist_threshold * dist_threshold;
    double total_length = 0.0;
    bool initial_cut = false;

    auto it = global_plan.begin();
    for (; it != global_plan.end(); ++it) {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;

      if (dist_sq < dist_behind_robot_sq && !initial_cut) {
        global_plan.erase(global_plan.begin(), it);
        initial_cut = true; // Start to count the distance from this point.
        it = global_plan.begin(); // Reset iterator to the new beginning.
        if (it != global_plan.end())
          continue;
        else
          break;
      }

      if (initial_cut) {
        if (it != global_plan.begin()) {
          total_length += calculateDistance((it-1)->pose, it->pose);
          if (total_length > max_path_length || dist_sq > dist_threshold_sq) {
            global_plan.erase(it, global_plan.end());
            break;
          }
        }
      }
    }

    if (!initial_cut) // No cut made, no poses are behind the robot
      return false;

  } catch (const tf2::TransformException& ex) {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }

  return true;
}

// Convert a ROS path from geometry_msgs::PoseStamped format to a simpler std::vector<std::pair<double, double>> format by reference.
void PsoLqrPlanner::convertPlan(const std::vector<geometry_msgs::PoseStamped>& pose_stamped_vector,
                 std::vector<std::pair<double, double>>& path) 
{
    path.clear();  // Clear any existing content in the output vector
    for (const auto& pose_stamped : pose_stamped_vector) {
        double x = pose_stamped.pose.position.x;
        double y = pose_stamped.pose.position.y;
        path.push_back(std::make_pair(x, y));
    }
}

// Check if the checkpoint is an obstacle
bool PsoLqrPlanner::collision(double x, double y)
{
  unsigned int mx,my;
  if(!worldTolocalMap(x,y,mx,my))
    return true;
  if ((mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY()))
    return true;
  if (costmap_->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return true;
  return false;
}

bool PsoLqrPlanner::isLineFree(const std::pair<double, double> p1, const std::pair<double, double> p2)
{
  std::pair<double, double> ptmp;
  ptmp.first = 0.0;
  ptmp.second = 0.0;

  double dist = sqrt( (p2.second-p1.second) * (p2.second-p1.second) +
                      (p2.first-p1.first) * (p2.first-p1.first) );
  if (dist < this->resolution_)
  {
      return true;
  }
  else
  {
    int value = int(floor(dist/this->resolution_));
    double theta = atan2(p2.second - p1.second,
                          p2.first - p1.first);
    int n = 1;
    for (int i = 0;i < value; i++)
    {
      ptmp.first = p1.first + this->resolution_*cos(theta) * n;
      ptmp.second = p1.second + this->resolution_*sin(theta) * n;
      if (collision(ptmp.first, ptmp.second))
        return false;
      n++;
    }
    return true;
  }
}


// Extract critical path points
void PsoLqrPlanner::cutPathPoint(std::vector< std::pair<double, double> >& plan, int MinPoints)
{
    size_t current_index = 0;
    size_t check_index = current_index+2;
    while(ros::ok())
    {
      if(( current_index >= plan.size()-2 )||(plan.size() < MinPoints))
        return;


      if(isLineFree(plan[current_index], plan[check_index]) ) 
      {

        std::vector< std::pair<double, double> >::iterator it = plan.begin()+ static_cast<int>(current_index + 1) ;
        if(check_index-current_index-1 == 1)
        {
          plan.erase(it);
        }
        else
        {
          plan.erase(it,it+static_cast<int>( check_index-current_index-1) );
          check_index = current_index + 2;
        }
      }
      else
      {
        if(check_index < plan.size()-1 )
          check_index++;
        else
        {
          current_index++;
          check_index = current_index + 2;
        }
      }
    }
}

bool PsoLqrPlanner::getPlanFromPath(std::vector< std::pair<int, int> >& path, std::vector<geometry_msgs::PoseStamped>& plan)
{
  plan.clear();
  unsigned int mx = 0,my = 0;
  double wx, wy;

  for (int i = 0; i < path.size(); i++)
  {
    mx=static_cast<unsigned int>(path[i].first);
    my=static_cast<unsigned int>(path[i].second);
    localmapToWorld(mx,my,wx,wy);
    // coding as message type
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }

  return !plan.empty();
}


// Set the plan that the controller is following
bool PsoLqrPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  ROS_INFO("Got new plan");

  // set new plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset plan parameters
  if (goal_x_ != global_plan_.back().pose.position.x || goal_y_ != global_plan_.back().pose.position.y)
  {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_rpy_ = getEulerAngles(global_plan_.back());
    goal_reached_ = false;
  }

  return true;
}

// Check if the goal pose has been achieved
bool PsoLqrPlanner::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("LQR planner has not been initialized");
    return false;
  }

  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    return true;
  }
  return false;
}


// Tranform from in_pose to out_pose with out frame using tf
void PsoLqrPlanner::transformPose(tf2_ros::Buffer* tf, const std::string out_frame,
                                 const geometry_msgs::PoseStamped& in_pose, geometry_msgs::PoseStamped& out_pose) const
{
  if (in_pose.header.frame_id == out_frame)
    out_pose = in_pose;

  tf->transform(in_pose, out_pose, out_frame);
  out_pose.header.frame_id = out_frame;
}

// Get the Euler Angles from PoseStamped
Eigen::Vector3d PsoLqrPlanner::getEulerAngles(geometry_msgs::PoseStamped& ps)
{
  tf2::Quaternion q(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w);
  tf2::Matrix3x3 m(q);

  double roll(0.0), pitch(0.0), yaw(0.0);
  m.getRPY(roll, pitch, yaw);

  return Eigen::Vector3d(roll, pitch, yaw);
}



// Transforms a global plan into a local coordinate frame.
bool PsoLqrPlanner::transformGlobalPlanToLocal( const tf2_ros::Buffer& tf_buffer, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                                const geometry_msgs::PoseStamped robot_pose_odom, const std::string& local_frame, std::vector<geometry_msgs::PoseStamped>& local_plan)
{
    if (global_plan.empty()) {
        ROS_ERROR("Received empty global plan");
        return false;
    }

    local_plan.clear();
    geometry_msgs::TransformStamped transform_to_local_frame;

    try {   transform_to_local_frame = tf_buffer.lookupTransform(local_frame, ros::Time(0),
                                                                 robot_pose_odom.header.frame_id, ros::Time(0),
                                                                 robot_pose_odom.header.frame_id, ros::Duration(1.0));
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("TF error in transforming global plan to local frame: %s", ex.what());
        return false;
    }

    for (const auto& pose : global_plan) 
    {
        geometry_msgs::PoseStamped local_pose;
        tf2::doTransform(pose, local_pose, transform_to_local_frame);
        local_plan.push_back(local_pose);
    }

    return true;
}

// world To localMap
bool PsoLqrPlanner::worldTolocalMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
  if (wx < origin_x_ || wy < origin_y_)
    return false;

  mx = (int)((wx - origin_x_) / resolution_);
  my = (int)((wy - origin_y_) / resolution_);

  if (mx < nx_ && my < ny_)
    return true;

  return false;
}

// localmap to World
void PsoLqrPlanner::localmapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;  //+0.5是为了取栅格中点坐标
  wy = origin_y_ + (my + 0.5) * resolution_;
}



/**
 * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
 * @param cmd_vel will be filled with the velocity command to be passed to the robot base
 * @return true if a valid trajectory was found, else false
 */
bool PsoLqrPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("PSOLQR planner has not been initialized");
    return false;
  }
  
  // update origin_x_、origin_y_ 
  origin_x_ = costmap_->getOriginX(), origin_y_ = costmap_->getOriginY();
  //ROS_INFO("origin_x_ %lf,   origin_y_: %lf", origin_x_,origin_y_);

  // get robot position in global frame
  geometry_msgs::PoseStamped robot_pose_odom, robot_pose_map;
  costmap_ros_->getRobotPose(robot_pose_odom);
  transformPose(tf_, map_frame_, robot_pose_odom, robot_pose_map);
  
  if (global_plan_.size()>2)
  {
      //Save Global Target Point
      global_target_ = global_plan_.back();
      // prune global plan to cut off parts of the past (spatially before the robot)
      pruneGlobalPath(*tf_, robot_pose_odom, global_plan_, global_dist_behind_robot_,  global_max_path_length_, global_dist_threshold_);
      // Convert global path to local coordinate system
      // transformGlobalPlanToLocal(*tf_, global_plan_, robot_pose_odom, base_frame_, global_plan_b);
      // Convert a ROS path from geometry_msgs::PoseStamped format to a simpler std::vector<std::pair<double, double>> format by reference.
      convertPlan(global_plan_, global_path_);
      // Extract critical path points
      cutPathPoint(global_path_,Min_num_points_);

      if (global_path_.size()>2)
      {
        //Convert to map coordinate system
        std::pair<int, int> temp_node;
        unsigned int mx = 0,my = 0;
        global_path_map_.clear();

        for (const auto& point : global_path_) 
        {
          if (worldTolocalMap(point.first, point.second, mx, my)) 
          {
            temp_node.first =static_cast<int>(mx);
            temp_node.second=static_cast<int>(my);
            global_path_map_.push_back(temp_node);
          } 
          else 
          {
            ROS_INFO("A path point conversion failed");
            continue;
          }
        }

        if (global_path_map_.size()>2)
        {

          local_path_.clear();
          // Call PSO planner for local planning
          //double start_time = ros::Time::now().toSec();
          bool path_found = pso_planner_->plan(costmap_->getCharMap(), global_path_map_, local_path_,origin_x_,origin_y_);


          if (path_found)
          {
            if (getPlanFromPath(local_path_, local_plan_))
            {
              //geometry_msgs::PoseStamped goal_copy = goal;
              //goal_copy.header.stamp = ros::Time::now();
              //plan.push_back(goal_copy);

              // path point attitude correction
              optimizationOrientation(local_plan_);
            }
            else
            { 
              local_plan_=global_plan_;
              ROS_ERROR("Failed to get a plan from path when a legal path was found. This shouldn't happen.");
            }
          }
          else
          { 
            local_plan_=global_plan_;
            ROS_ERROR("Failed to get a path.");
          }
        }
        else
        {
          local_plan_=global_plan_;
        }
      }
      else
      {
        local_plan_=global_plan_;
      }

  }
  else
  {
    local_plan_=global_plan_;
  }

  // transform global plan to robot frame
  // std::vector<geometry_msgs::PoseStamped> local_plan = prune(robot_pose_map);

  // Calling LQR controller to calculate control instructions
  bool obtain_control = lqr_controller_->lqr_control(robot_pose_map, local_plan_, global_target_,goal_rpy_, goal_reached_, d_t_, cmd_vel,lookahead_pt_);
  
  if(!obtain_control)
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }

  //std::cout<<"frame_id:"<<frame_id_<<std::endl;

  // publish visulization plan
  nav_msgs::Path path_pose;
  path_pose.header.frame_id = frame_id_;
  path_pose.header.stamp = ros::Time::now();
  path_pose.poses = local_plan_;
  plan_pub_.publish(path_pose);

  // publish lookahead pose
  target_pt_pub_.publish(lookahead_pt_);


  // publish robot pose
  current_pose_pub_.publish(robot_pose_map);

  return true;
}



}  // namespace psolqr_planner
