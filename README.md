# psolqr_local_planner

![psolqrLocalPLanner](assets/psolqrLocalPLanner.png)

<p align="center">
    <img width="100px" height="20px" src="https://img.shields.io/badge/Ubuntu-20.04-orange?logo=Ubuntu&Ubuntu-20.04"
        alt="ubuntu" />
    <img width="100px" height="20px" src="https://img.shields.io/badge/ROS-noetic-blue?logo=ROS&ROS=noetic" alt="ROS" />
</p>

## 1. Introduction

Lightweight ROS Local Path Planner Plugin with PSO and LQR:

This ROS plugin offers a lightweight solution for local path planning. It employs the Particle Swarm Optimization (PSO) algorithm for precise local path planning and leverages the Linear Quadratic Regulator (LQR) algorithm to achieve accurate trajectory tracking control."

"In the design of our PSOLQR local path planner, we adopted a decoupled architecture for planning and control. The core processes of the planning and control algorithms are implemented separately in pso_optimizer.cpp and lqr_controller.cpp, respectively. These components offer reserved interfaces for quick and easy access to planning and control functions. This modular approach allows for convenient changes to planning or control algorithms without needing to redesign the local planner, offering excellent scalability."

An illustration of the system's performance is shown below：


### (1) Diagram 1:

<div align="center">
  <img src="assets/psolqrPlanner.gif" alt="demo1.gif">
</div>

### (2) Diagram 2:

<div align="center">
  <img src="assets/psolqrPlanner2.gif" alt="demo2.gif">
</div>


###


The development of this open-source repository has been relatively short. We've completed the design and implementation of the entire framework, ensuring it functions properly. However, due to time constraints, many intermediate processes and details have not been thoroughly validated to determine whether they meet the intended design expectations and effectiveness. Experimentally, the results have not reached the desired goals, indicating a need for further optimization and adjustments. Unfortunately, for a significant period thereafter, I'll be occupied with other important matters, forcing the optimization of this project to be temporarily suspended. We eagerly welcome interested contributors to optimize and enhance it. Of course, in the future, I may revisit it for optimization.



## 2. How to Use


You can employ this plugin within the ROS navigation package by configuring the local path planner plugin to `psolqr_planner/PsoLqrPlanner` in the launch file where the 'move_base' node is situated. Additionally, load the parameter configuration file `psolqr_planner_params.yaml`. An example is provided below:

```bash
<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">

    <!-- Load other parameter configuration files -->
    ...
    ...
    ...

    <!-- Load PSOLQR Local Planner Parameters -->
    <rosparam file="$(find psolqr_local_planner)/config/psolqr_planner_params.yaml" command="load" />

    <!-- Set Local Path Planner Plugin -->
    <param name="base_global_planner" value="psolqr_planner/PsoLqrPlanner" />

    <!-- Set other parameters such as global path planner plugin -->
    ...
    ...
    ...

</node>
```

## 3. Parameter Descriptions

The PSOLQR planner includes several key configuration parameters that allow users to tailor the performance according to their needs. Below are detailed descriptions of the configurable parameters:

### PSO (Particle Swarm Optimization) Parameters

- `n_particles`: Number of particles in the swarm.
- `n_inherited`: Number of inherited particles.
- `pointNum`: Number of position points contained in each particle.
- `max_speed`: Maximum speed of particle movement.
- `w_inertial`: Inertial weight.
- `w_social`: Social influence weight.
- `w_cognitive`: Cognitive weight.
- `obs_factor`: Obstacle factor.
- `pso_state_init_ratio`: Initialization ratio for state points.
- `pso_disturbance_state_init_ratio`: Initialization ratio for disturbed state points.
- `pub_particles`: Whether to publish particle information.
- `pso_max_iter`: Maximum number of iterations for PSO.

### Global Path Clipping Parameters

- `global_dist_behind_robot`: Distance to maintain behind the robot.
- `global_max_path_length`: Maximum allowed length of the path remaining after clipping.
- `global_dist_threshold`: Maximum distance a pose can be from the current robot's pose to remain in the plan.

### LQR (Linear Quadratic Regulator) Parameters

- `goal_dist_tolerance`: Tolerance for reaching the goal distance.
- `rotate_tolerance`: Tolerance for rotation angle.
- `lqr_max_iter`: Maximum number of iterations for LQR computation.
- `lqr_eps_iter`: Precision for LQR iterations.
- `max_v`: Maximum linear velocity.
- `min_v`: Minimum linear velocity.
- `max_w`: Maximum angular velocity.
- `min_w`: Minimum angular velocity.
- `lookahead_time`: Lookahead time.
- `min_lookahead_dist`: Minimum lookahead distance.
- `max_lookahead_dist`: Maximum lookahead distance.
- `Q_matrix_diag`: Weight matrix for penalizing state error while tracking.
- `R_matrix_diag`: Weight matrix for penalizing input error while tracking.

---

For more information, please refer to the file psolqr_planner_params.yaml

---

## 4. File Descriptions

### Core Components

- **`psolqr_planner.h/cpp`**: Implements the PSO_LQR local planner class. These files integrate the functionalities of the PSO optimizer and the LQR controller into a local path planning plugin for ROS. They handle the setup and updates required for local path planning using the combined strategies of PSO and LQR.

- **`pso_optimizer.h/cpp`**: Contains the implementation of the Particle Swarm Optimization (PSO) planner. These files define the optimization strategies for the particles, manage particle properties like position and velocity, and handle the convergence of the swarm towards the optimal path under given constraints.

- **`lqr_controller.h/cpp`**: Houses the abstract LQR controller class. This component calculates the optimal control commands based on the current robot state and the desired trajectory, using the Linear Quadratic Regulator approach for minimized error.

- **`pso_trajectory.h/cpp`**: Consists of the trajectory generation class that uses quasi-uniform B-spline curves to generate smooth trajectories from given waypoints. These files play a crucial role in defining the path that the robot will follow, ensuring smoothness and feasibility with respect to the robot's dynamics.

### Utility and Helper Components

- **`psolqr_planner_plugin.xml`**: Defines the ROS pluginlib class loader details for the PSO_LQR planner, allowing it to be dynamically loaded along with specified parameters.

Each of these files is integral to the functioning of the PSO_LQR planner, playing a specific role in either planning, control, or utility functions.

--- 


## 5. Acknowledgments

We extend our heartfelt thanks to Mr. Yang Haodong, the original author of the `lqr_planner` from the open-source project `ros_motion_planning`. His work has significantly influenced the implementation of the LQR controller in our project. The original project can be accessed at [ros_motion_planning](https://github.com/ai-winter/ros_motion_planning).

In this project, we have taken the foundation laid by Mr. Yang and evolved it by redesigning the framework structure to establish an independent LQR control library. This new structure is not dependent on the `local_planner` package and allows for easy calculation of control commands using the `lqr_control()` function. This redesign facilitates the straightforward invocation of this control library, enhancing its applicability and integration into diverse robotic systems.

We are deeply grateful for Mr. Yang Haodong's spirit of sharing and contribution, which has allowed us to further develop and improve upon his exemplary work.

--- 





