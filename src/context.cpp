#include "context.h"
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>

#include <cmath>
#include <random>

// utility function to test for a collision
bool CS3891Context::is_colliding( const vertex& q ) const {
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", q );

  if( getPlanningScene()->isStateColliding( robotstate, "manipulator", false ) )
    { return true; }
  else
    { return false; }
}

// utility function to interpolate between two configurations
CS3891Context::vertex CS3891Context::interpolate( const CS3891Context::vertex& qA,
						const CS3891Context::vertex& qB,
						double t ){
  CS3891Context::vertex qt( qA.size(), 0.0 );
  for( std::size_t i=0; i < qt.size(); i++ )
    { qt[i] = ( 1.0 - t )*qA[i] + t*qB[i]; }
  return qt;
}

CS3891Context::CS3891Context( const robot_model::RobotModelConstPtr& robotmodel,
			    const std::string& name, 
			    const std::string& group, 
			    const ros::NodeHandle& nh ) :
  planning_interface::PlanningContext( name, group ),
  robotmodel( robotmodel ){

  }

CS3891Context::~CS3891Context(){}

CS3891Context::vertex CS3891Context::random_sample(const CS3891Context::vertex& q_start, const CS3891Context::vertex& q_goal, size_t attemptLimit, bool forceBias) const {
  if (attemptLimit < 1) {
    return q_start;
  }
  vertex q_rand;
  vertex differenceVector;
	double bias = 0.05;
	double stepSize = 3;
	double goalDistance = distance(q_start, q_goal);

  if (goalDistance < stepSize && is_subpath_collision_free(q_start, q_goal)) {
	  return q_goal;
  }

  if ((rand() < bias * RAND_MAX)|| forceBias ) {
    vertex midPoint;
    for (size_t i = 0; i < q_start.size(); ++i) {
      double temp1 = (q_start[i] + q_goal[i]) / 2;

      if (temp1 < -M_PI) {
        temp1 += 2 * M_PI;
      }
      else if (temp1 >= M_PI) {
        temp1 -= 2 * M_PI;
      }
      midPoint.push_back(temp1);
    }

    if (!is_colliding(midPoint) && is_subpath_collision_free(q_start, midPoint)) {
      return random_sample(midPoint, q_goal, attemptLimit - 1, true);
    }
    else {
      return random_sample(q_start, q_goal, attemptLimit - 1, false);
    }
  }

  for (size_t i = 0; i < q_start.size(); i++) {
    double temp = q_start[i] + (stepSize * (double(rand()) - double(RAND_MAX)/2.0) / double(RAND_MAX));
    if (temp < -M_PI) {
      temp += 2 * M_PI;
    }
    else if (temp >= M_PI) {
      temp -= 2 * M_PI;
    }

    q_rand.push_back(temp);
  }

  if (!is_colliding(q_rand) && is_subpath_collision_free(q_start, q_rand)) {
	  return q_rand;
  }
  else {
    return random_sample(q_start, q_goal, attemptLimit - 1, false);
  }
}

double CS3891Context::distance(const CS3891Context::vertex& q1, const CS3891Context::vertex& q2) const{
  double d = 0;
  double diff, wrappedDiff;

  // Euclidian Distance with 2 * pi wrap
  for (size_t i = 0; i < q1.size(); i++) {

	if (q1[i] > q2[i]) {
	  diff = q1[i] - q2[i];
	}
	else {
	  diff = q2[i] - q1[i];
	}

	wrappedDiff = 2 * M_PI - diff;

	if (diff < wrappedDiff) {
	  d += diff * diff;
	}
	else {
	  d += wrappedDiff * wrappedDiff;
	}
  }

  return sqrt(d);
}

// TODO
size_t CS3891Context::nearest_configuration(const CS3891Context::vertex& q_rand, const size_t lastBestIndex) {
  CS3891Context::vertex bestPoint = nodes[lastBestIndex].point;
  double bestDistance = distance(bestPoint, q_rand);
  size_t bestIndex = lastBestIndex;

  // printf("jjjjjjjjjjjjjjjjjjj %d \t \t %d\n", nodes.size(), lastBestIndex);
  for (size_t i = nodes.size() - int(sqrt(nodes.size())); i < nodes.size(); ++i) {
    double dist = distance(nodes[i].point, q_rand);
    if (dist < bestDistance) {
      bestDistance = dist;
      bestIndex = i;
      // printf("jjjjjjjjjjjjjjjjjjj %d \t \t %d\n", i, nodes[i].parentIndex);
    }
  }

  // printf("index %d \t\t %d\n", bestIndex, nodes.size());
  return bestIndex;
}

bool CS3891Context::is_subpath_collision_free( const CS3891Context::vertex& q_near, const CS3891Context::vertex& q_new) const{
  double minDistance = 0.01;

  path toCheck;
  path nextCheck;

  toCheck.push_back(q_near);
  toCheck.push_back(q_new);

  while (true) {
    double dist = distance(toCheck[0], toCheck[1]);
      // printf("HERRRRRRRRRRRRRRRRRRRRRRRR\t%f\t %d\n", dist, toCheck.size());

    if (minDistance > dist) {
      return !is_colliding(toCheck[1]);
    }
    
    vertex midPoint;
    for (size_t i = 1; i < toCheck.size(); ++i) {
      for (size_t j = 0; j < toCheck[i].size(); ++j) {
        double temp1 = (toCheck[i][j] + toCheck[i - 1][j]);
        double diff = std::abs(toCheck[i][j] - toCheck[i - 1][j]);

        if (diff < M_PI) {
          temp1 /= 2;
        }
        else {
          if (toCheck[i - 1][j] < 0) {
            temp1 = toCheck[i - 1][j] - (2 * M_PI - diff) / 2;
          }
          else  {
            temp1 = toCheck[i - 1][j] + (2 * M_PI - diff) / 2;
          }
        }

        if (temp1 < -M_PI) {
          temp1 += 2 * M_PI;
        }
        else if (temp1 >= M_PI) {
          temp1 -= 2 * M_PI;
        }

        midPoint.push_back(temp1);
      }

      if (is_colliding(midPoint)) {
        return false;
      }

      nextCheck.push_back(toCheck[i-1]); 
      nextCheck.push_back(midPoint);
      midPoint.clear();
    }

    nextCheck.push_back(toCheck[toCheck.size() - 1]);
    toCheck.clear();
    toCheck.swap(nextCheck);
  }
}

CS3891Context::path CS3891Context::search_path( const CS3891Context::vertex& q_init,
					      const CS3891Context::node& q_goal ){
  CS3891Context::path P;

  // TODO Once q_goal has been added to the tree, find the path (sequence of configurations) between
  // q_init and q_goal (hint: this is easier by using recursion).
  // printf("HERRRRRRRRRRRRRRRRRRRRRRRR\n");

  int i = q_goal.parentIndex;
  P.push_back(q_goal.point);
  // printf("search_path index %d\n", i);

  while (i > -1) {
    // printf("search_path index %d\n", i);.

    P.push_back(nodes[i].point);
    i = nodes[i].parentIndex;
  }
  printf("bbbbbbbbbbbbbbbbbbbbbbbbbbbb%d\n", P.size());

  return P;
}

// TODO
CS3891Context::path CS3891Context::rrt( const CS3891Context::vertex& q_init,
				      const CS3891Context::vertex& q_goal ){
  CS3891Context::path P;

  // TODO implement RRT algorithm and return the path (an ordered sequence of configurations).
  size_t lastBestIndex = 0;
  size_t initialRandomSize = 16;
  double minDistance = 0.01;
  double bestDistance = 1000;
  double bias = 0.05;

  node temp;
  temp.point = q_init;
  temp.parentIndex = -1;
  nodes.push_back(temp);

  for (size_t i = 0; i < initialRandomSize; ++i) {
    node temp2;
    temp2.point = random_sample(temp.point, q_goal, 20, false);
    temp2.parentIndex = 0;
    nodes.push_back(temp2);
  }

  printf("Starting\n\n\n\n\n");

  while (nodes.size() < 2000) {
    lastBestIndex = nearest_configuration(q_goal, lastBestIndex);
    CS3891Context::node point = nodes[lastBestIndex];
    double dist = distance(point.point, q_goal);
    size_t index;
    // printf("qqqqqqqqqqqqqqqqqqqqqqqqqqqq %f %f nodes size %d\n",bestDistance, dist, nodes.size() - 1);

    if (dist < bestDistance) {
      bestDistance = dist;
    }
      printf("closest distance %f, nodes size %d\n", bestDistance, nodes.size() - 1);

    if (dist < minDistance) {
      printf("found path\n");
      return search_path(q_init, point);
    }

    if (rand() < bias * RAND_MAX) {
      index = lastBestIndex;
    }
    else {
      index = nodes.size() - (rand() % int(sqrt(nodes.size()))) - 1;
    }

    for (size_t i = 0; i < int(sqrt(nodes.size())); ++i) {
      node temp3;
    //   printf("aaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
      temp3.point = random_sample(nodes[index].point, q_goal, 20, false);
    //   printf("ccccccccccccccccccccccccccc\n");
      temp3.parentIndex = index;
      nodes.push_back(temp3);
    }
  }

  return P;
}

// This is the method that is called each time a plan is requested
bool CS3891Context::solve( planning_interface::MotionPlanResponse &res ){

  // Create a new empty trajectory
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel, getGroupName()));
  res.trajectory_->clear();

  // copy the initial/final joints configurations to vectors qfin and qini
  // This is mainly for convenience.
  vertex q_init, q_goal;
  for( size_t i=0; i<robotmodel->getVariableCount(); i++ ){
    q_goal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
    q_init.push_back(request_.start_state.joint_state.position[i]);
  }

  // start the timer
  ros::Time begin = ros::Time::now();

  nodes.clear();
  path P = rrt( q_init, q_goal );

  // end the timer
  ros::Time end = ros::Time::now();

  // The rest is to fill in the animation. You can ignore this part.
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", q_init );

  for( std::size_t i=P.size()-1; i>=1; i-- ){
    for( double t=0.0; t<=1.0; t+=0.01 ){
      vertex q = interpolate( P[i], P[i-1], t );
      robotstate.setJointGroupPositions( "manipulator", q );
      res.trajectory_->addSuffixWayPoint( robotstate, 0.01 );
    }
  }
  
  // set the planning time
  ros::Duration duration = end-begin;
  res.planning_time_ = duration.toSec();
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  return true;
  
}

bool CS3891Context::solve( planning_interface::MotionPlanDetailedResponse &res )
{ return true; }

void CS3891Context::clear(){}

bool CS3891Context::terminate(){return true;}
