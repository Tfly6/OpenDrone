/**
 * ref : https://github.com/ethz-asl/mav_trajectory_generation/blob/master/mav_trajectory_generation_example/src/example_planner.cpp
 *
 * trajectory_generation
 *
 * @author TFly
 */


#include "mav_trajectory_generation_ros/trajectory_generation.h"

TrajectoryGeneration::TrajectoryGeneration(ros::NodeHandle& nh) :
    nh_(nh),
    max_v_(2.0),
    max_a_(2.0),
    dimension_(3),
    current_velocity_(Eigen::Vector3d::Zero()),
    current_pose_(Eigen::Affine3d::Identity()) {

    nh.param("max_v", max_v_, 2.0);
    nh.param("max_a", max_a_, 2.0);


  // create publisher for RVIZ markers
  pub_markers_ =
      nh.advertise<visualization_msgs::MarkerArray>("/trajectory_generation/trajectory_markers", 0);

    pub_trajectory_ =
        nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("/trajectory_generation/trajectory",
                                                              1);
    pub_trigger_ = nh.advertise<geometry_msgs::PoseStamped>("/waypoint_generator/traj_start_trigger", 0);

  // subscriber for Odometry
    sub_odom_ =
        nh.subscribe("/mavros/local_position/odom", 1, &TrajectoryGeneration::uavOdomCallback, this);

    sub_waypoint_ = nh.subscribe("/trajectory_generation/waypoint", 1, &TrajectoryGeneration::waypointCallback, this);

    nh.param("loop_trajectory", loop_trajectory_, false);
    nh.param("replan_ratio", replan_ratio_, 0.8);

    // 20Hz 定时检查是否需要重规划
    replan_timer_ = nh.createTimer(ros::Duration(0.05), &TrajectoryGeneration::checkAndReplan, this);
}

// Callback to get current Pose of UAV
void TrajectoryGeneration::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

  // store current position in our planner
  tf::poseMsgToEigen(odom->pose.pose, current_pose_);


  // store current vleocity
  tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
    // std::cout << "current_velocity: " << current_velocity_ << std::endl;
}

void TrajectoryGeneration::waypointCallback(const nav_msgs::Path::ConstPtr& msg) {
  waypoints_.clear();  // 清空之前的航点
  for(int i=0; i<msg->poses.size(); i++){
      waypoints_.push_back(msg->poses[i].pose);
  }
  ROS_INFO("Received %zu waypoints.", waypoints_.size());
  if(waypoints_.size() > 0){
    planTrajectory();
  }
}

void TrajectoryGeneration::planTrajectory() {
  mav_trajectory_generation::Vertex::Vector vertices;

  mav_trajectory_generation::Vertex start(dimension_), end(dimension_);

  start.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                        current_pose_.translation());
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      current_velocity_);

  vertices.push_back(start);
  for(int i = 0; i < waypoints_.size(); i++) {
    if(i == waypoints_.size()-1){
      if (loop_trajectory_) {
        // 循环模式：终点只约束位置，不强制速度为零，保持速度连续
        end.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                          Eigen::Vector3d(waypoints_[i].position.x,
                                          waypoints_[i].position.y,
                                          waypoints_[i].position.z));
      } else {
        end.makeStartOrEnd(Eigen::Vector3d(waypoints_[i].position.x,
                                          waypoints_[i].position.y,
                                          waypoints_[i].position.z), derivative_to_optimize_);
      }
    }
    else{
      mav_trajectory_generation::Vertex middle(dimension_);
      middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                          Eigen::Vector3d(waypoints_[i].position.x, 
                                          waypoints_[i].position.y,
                                          waypoints_[i].position.z));
      vertices.push_back(middle);
    }
  }
  vertices.push_back(end);

  mav_trajectory_generation::Trajectory trajectory;
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

  // 使用线性优化，速度远快于非线性优化，适合循环重规划场景
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension_);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
  opt.solveLinear();
  opt.getTrajectory(&trajectory);

  // 记录轨迹时长和起始时间，供循环重规划使用
  trajectory_duration_ = trajectory.getMaxTime();
  trajectory_start_time_ = ros::Time::now();
  trajectory_active_ = true;

  publishTrajectory(trajectory);
}

void TrajectoryGeneration::checkAndReplan(const ros::TimerEvent&) {
  if (!loop_trajectory_ || !trajectory_active_ || waypoints_.empty()) return;

  double elapsed = (ros::Time::now() - trajectory_start_time_).toSec();
  double remaining_ratio = 1.0 - elapsed / trajectory_duration_;

  // 当剩余时间比例低于阈值时触发重规划
  if (remaining_ratio <= (1.0 - replan_ratio_)) {
    ROS_INFO("[TrajectoryGeneration] Loop replan triggered (elapsed=%.2fs / %.2fs)",
             elapsed, trajectory_duration_);
    trajectory_active_ = false;  // 防止重复触发
    planTrajectory();
  }
}

bool TrajectoryGeneration::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance =
      0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "map";

  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_markers_.publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "map";
  pub_trajectory_.publish(msg);

  return true;
}

void TrajectoryGeneration::triggerWaypoints() {
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  tf::poseEigenToMsg(current_pose_, pose.pose);
  // pose.pose.position = toGeometryMsg(current_pose_);
  pub_trigger_.publish(pose);
}

// Method to set maximum speed.
// void TrajectoryGeneration::setMaxSpeed(const double max_v) {
//   max_v_ = max_v;
// }

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
// bool TrajectoryGeneration::planTrajectory(const Eigen::VectorXd& goal_pos,
//                                     const Eigen::VectorXd& goal_vel,
//                                     mav_trajectory_generation::Trajectory* trajectory) {


//   // 3 Dimensional trajectory => through carteisan space, no orientation
//   const int dimension = 3;

//   // Array for all waypoints and their constrains
//   mav_trajectory_generation::Vertex::Vector vertices;

//   // Optimze up to 4th order derivative (SNAP)
//   const int derivative_to_optimize =
//       mav_trajectory_generation::derivative_order::SNAP;

//   // we have 2 vertices:
//   // Start = current position
//   // end = desired position and velocity
//   mav_trajectory_generation::Vertex start(dimension), end(dimension);


//   /******* Configure start point *******/
//   // set start point constraints to current position and set all derivatives to zero
//   // 将起始点约束设置为当前位置，并将所有导数设为零
// //   start.makeStartOrEnd(current_pose_.translation(),
// //                        derivative_to_optimize);
//     start.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
//                         current_pose_.translation());

//   // set start point's velocity to be constrained to current velocity
//   start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
//                       current_velocity_);


//   // add waypoint to list
//   vertices.push_back(start);


//   /******* Configure end point *******/
//   // set end point constraints to desired position and set all derivatives to zero
//   end.makeStartOrEnd(goal_pos,
//                      derivative_to_optimize);

//   // set start point's velocity to be constrained to current velocity
// //   end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
// //                      goal_vel);

//   // add waypoint to list
//   vertices.push_back(end);

//   ROS_INFO_STREAM(vertices);

//   // setimate initial segment times
//   std::vector<double> segment_times;
//   segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

//   // Set up polynomial solver with default params
//   mav_trajectory_generation::NonlinearOptimizationParameters parameters;

//   // set up optimization problem
//    const int N = 10;
//   mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
//   opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

//   // constrain velocity and acceleration
//   opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
//   opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

//   // solve trajectory
//   opt.optimize();

//   // get trajectory as polynomial parameters
//   opt.getTrajectory(&(*trajectory));
//     // mav_trajectory_generation::PolynomialOptimization<N>* opt_ptr_ =
// 	// 		new mav_trajectory_generation::PolynomialOptimization<N>(dimension);
// 	// opt_ptr_->setupFromVertices(vertices, segment_times, derivative_to_optimize);
// 	// opt_ptr_->solveLinear();
// 	// // opt_ptr_->getSegments(&segments);
// 	// opt_ptr_->getTrajectory(&(*trajectory));

//   return true;
// }


