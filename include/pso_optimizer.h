/***********************************************************
 *
 * @file: pso_optimizer.h
 * @breif: Contains the Particle Swarm Optimization(PSO) planner class
 * @author: Jing Zongxin
 * @update: 2024-4-19
 * @version: 1.2
 *
 * Copyright (c) 2024，Jing Zongxin
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/

#ifndef PSO_OPTIMIZER_H
#define PSO_OPTIMIZER_H

#include <random>
#include <thread>
#include <mutex>
#include <vector>
#include "pso_trajectory.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using PositionSequence = std::vector<std::vector<std::pair<int, int>>>;

namespace psolqr_planner
{
  struct Particle
  {
    std::vector<std::pair<int, int>> position;               // Particle position
    std::vector<std::pair<int, int>> velocity;                           // Particle velocity
    double fitness;                                          // Particle fitness
    std::vector<std::pair<int, int>> personal_best_pos;      // Personal best position in iteration
    double personal_best_fitness;                            // Personal best fitness in iteration

    Particle() = default;  

    Particle(const std::vector<std::pair<int, int>>& initial_position,
            const  std::vector<std::pair<int, int>>& initial_velocity,
            double initial_fitness)
        : position(initial_position),
          velocity(initial_velocity),
          fitness(initial_fitness),
          personal_best_pos(initial_position),
          personal_best_fitness(initial_fitness)
    {
    }
  };

  /**
   * @brief Class for objects that plan using the PSO algorithm
   */

  class PSO 
  {
  public:
    /**
     * @brief Construct a new PSO object
     * @param nx            pixel number in costmap x direction
     * @param ny            pixel number in costmap y direction
     * @param resolution    costmap resolution
     * @param origin_x      origin coordinate in the x direction.
     * @param origin_y      origin coordinate in the y direction.
     * @param n_particles	  number of particles
     * @param n_inherited   number of inherited particles
     * @param pointNum      number of position points contained in each particle
     * @param w_inertial	  inertia weight
     * @param w_social		  social weight
     * @param w_cognitive	  cognitive weight
     * @param obs_factor    obstacle factor(greater means obstacles)
     * @param max_speed		  The maximum movement speed of particles
     * @param state_init_ratio	             State point initialization ratio
     * @param disturbance_state_init_ratio   Random disturbance state point initialization ratio
     * @param pub_particles Boolean flag to publish particles.
     * @param max_iter		  maximum iterations
     */
    PSO(int nx, int ny, double resolution, double origin_x, double origin_y, int n_particles,
        int n_inherited, int pointNum , double w_inertial, double w_social, double w_cognitive, 
        double obs_factor,int max_speed, double state_init_ratio, double disturbance_state_init_ratio, 
        bool pub_particles,int max_iter);
    
    
    ~PSO();

  /**
   * @brief PSO implementation
   * @param global_costmap global costmap
   * @param path_g         Key points in the map coordinate system
   * @param path           optimal path consists of Node
   * @param origin_x       current local map origin x 
   * @param origin_y       current local map origin y
   * @return  true if path found, else false
   */
  bool plan(const unsigned char* global_costmap, std::vector< std::pair<int, int>>& path_g, std::vector< std::pair<int, int>>& path, double origin_x, double origin_y);

    
    /**
     * @brief Generate an initial position point sequence 
     * @param initialPositions The initial position sequence of particle swarm
     * @param start_d        starting point
     * @param goal_d         Target point
     * @param Keypoints      Key points
     */
    void generateInitialPositions(PositionSequence &initialPositions,const std::pair<double, double> start_d,const std::pair<double, double> goal_d, const std::vector< std::pair<int, int>> Keypoints);

    /**
     * @brief Calculate Obstacle avoidance cost
     * @param global_costmap   global costmap
     * @param pso_path         Path to perform collision detection
     * @return  The collision cost of the path
     */
    double ObstacleCost(const unsigned char* global_costmap,const std::vector<std::pair<double, double>>& pso_path);

    /**
     * @brief A function to update the particle velocity
     * @param particle     Particles to be updated for velocity
     * @param global_best  Global optimal particle
     * @param gen          randomizer
     */
    void updateParticleVelocity(Particle& particle,const Particle& global_best,std::mt19937& gen); 

    /**
     * @brief A function to update the particle position
     * @param particle     Particles to be updated for velocity
     */
    void updateParticlePosition(Particle& particle);

    
    /**
     * @brief Particle update optimization iteration
     * @param particle       Particles to be updated for velocity
     * @param best_particle  Global optimal particle
     * @param start_d        starting point
     * @param goal_d         Target point
     * @param index_i        Particle ID
     * @param global_costmap global costmap
     * @param gen            randomizer
     */
    void optimizeParticle(Particle& particle, Particle& best_particle, const unsigned char* global_costmap, 
                               const std::pair<double, double>& start_d, const std::pair<double, double>& goal_d,
                               const int& index_i,std::mt19937& gen) ;
    
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
     * @brief Custom comparison function for ascending order.
     * @param a   The first element to be compared.
     * @param b   The second element to be compared.
     * @return bool  True if 'a' is less than 'b', indicating ascending order; otherwise, false.
     */
    static bool ascendingOrder(int a, int b) { return a < b;}

    /**
     * @brief Custom comparison function for descending order.
     * @param a   The first element to be compared.
     * @param b   The second element to be compared.
     * @return bool  True if 'a' is greater than 'b', indicating descending order; otherwise, false.
     */
    static bool descendingOrder(int a, int b){ return a > b;}


    /**
     * @brief Publishes particle markers based on given positions and index.
     * @param positions   Vector of pairs representing positions (x, y) of particles.
     * @param index       Index identifier for the particles.
     */
    void publishParticleMarkers(const std::vector<std::pair<int, int>>& positions, const int& index);

    /**
     * @brief Transform from grid map(x, y) to grid index(i)
     * @param x grid map x
     * @param y grid map y
     * @return  index
     */
    int grid2Index (int x, int y);


  protected:
    int nx_,ny_,ns_;              // the number of grids on the x-axis and y-axis of the map, as well as the total number of grids   
    double resolution_;           // map resolution
    double origin_x_,origin_y_;   // origin coordinate in the x、y direction.
    bool pub_particles_;          // boolean flag to publish particles.
    int max_iter_;                // maximum iterations
    int n_particles_;             // number of particles
    int n_inherited_;             // number of inherited particles
    int pointNum_;                // number of position points contained in each particle
    double w_inertial_, w_social_, w_cognitive_;   // Weight coefficients for fitness calculation: inertia weight, social weight, cognitive weight
    double obs_factor_;           // obstacle factor(greater means obstacles)
    int max_speed_;               // The maximum velocity of particle motion
    double state_init_ratio_;               // Set state point initialization ratio
    double disturbance_state_init_ratio_;   // Set Random disturbance state point initialization ratio

  private:
    unsigned char lethal_cost_=253;                 //lethal cost
    ros::Publisher particle_pub;                    //The publisher of real-time particle visualization 
    int GlobalBest_particle_;                       //The ID of the globally optimal particle
    std::mutex particles_lock_;                     //thread lock
    std::vector<Particle> inherited_particles_;     //inherited particles
    trajectoryGeneration path_generation;           //Path generation
  };

}  // namespace global_planner
#endif
