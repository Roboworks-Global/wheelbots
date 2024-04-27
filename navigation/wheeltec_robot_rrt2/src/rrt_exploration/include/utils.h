#ifndef functions_H
#define functions_H
// #include "rclcpp/rclcpp.hpp"
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.h"
// rdm class, for gentaring random flot numbers
class rdm{
int i;
public:
rdm();
double randomize();
};

//norm function prototype
double norm( const std::vector<double> & , const std::vector<double> &);

//sign function prototype
double sign(double );

//Nearest function prototype
std::vector<double> Nearest(  std::vector< std::vector<double>  > , std::vector<double> );

//Steer function prototype
std::vector<double> Steer(  std::vector<double>, std::vector<double>, double );

//gridValue function prototype
int grid_value(nav_msgs::msg::OccupancyGrid &,std::vector<double>);

//ObstacleFree function prototype
char ObstacleFree(std::vector<double> , std::vector<double> & , nav_msgs::msg::OccupancyGrid);

double checkAround(const nav_msgs::msg::OccupancyGrid &mapData, const std::vector<double> &point, const double &r);

int index_of_point(const nav_msgs::msg::OccupancyGrid &mapData, const std::vector<double> &point);

std::vector<double> point_of_index(const nav_msgs::msg::OccupancyGrid &mapData, const int &i);


double information_gain(const nav_msgs::msg::OccupancyGrid &mapData, const std::vector<double> &point, const double &r);

int discount(const nav_msgs::msg::OccupancyGrid &,
             const std::vector<double> &,
             const std::vector<double> &, const double &);
#endif
