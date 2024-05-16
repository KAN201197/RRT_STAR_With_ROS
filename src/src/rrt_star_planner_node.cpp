#include "rrt_planner/rrt_planner.h"

int main(int argv, char ** argc)
{
  ros::init(argv, argc, "rrt_star_planner");
  ros::NodeHandle node;
  new rrt_planner::RRTPlanner(&node);
}
