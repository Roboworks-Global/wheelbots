#include "utils.h"

// rdm class, for gentaring random flot numbers
rdm::rdm() { i = time(0); }
double rdm::randomize()
{
  i = i + 1;
  srand(i);
  return double(rand()) / double(RAND_MAX);
}

//sign function
double sign(double n)
{
  if (n < 0.0)
  {
    return -1.0;
  }
  else
  {
    return 1.0;
  }
}

//Nearest function
std::vector<double> Nearest(std::vector<std::vector<double>> V, std::vector<double> x)
{

  double min = norm(V[0], x);
  int min_index;
  double temp;

  for (int j = 0; j < V.size(); j++)
  {
    temp = norm(V[j], x);
    if (temp <= min)
    {
      min = temp;
      min_index = j;
    }
  }

  return V[min_index];
}

//Steer function
std::vector<double> Steer(std::vector<double> x_nearest, std::vector<double> x_rand, double eta)
{
  std::vector<double> x_new;

  if (norm(x_nearest, x_rand) <= eta)
  {
    x_new = x_rand;
  }
  else
  {

    double m = (x_rand[1] - x_nearest[1]) / (x_rand[0] - x_nearest[0]);

    x_new.push_back((sign(x_rand[0] - x_nearest[0])) * (sqrt((pow(eta, 2)) / ((pow(m, 2)) + 1))) + x_nearest[0]);
    x_new.push_back(m * (x_new[0] - x_nearest[0]) + x_nearest[1]);

    if (x_rand[0] == x_nearest[0])
    {
      x_new[0] = x_nearest[0];
      x_new[1] = x_nearest[1] + eta;
    }
  }
  return x_new;
}

//grid_value function
int grid_value(nav_msgs::msg::OccupancyGrid &mapData, std::vector<double> Xp)
{

  double resolution = mapData.info.resolution;
  double Xstartx = mapData.info.origin.position.x;
  double Xstarty = mapData.info.origin.position.y;

  double width = mapData.info.width;
  std::vector<signed char> Data = mapData.data;

  //returns grid value at "Xp" location
  //map data:  100 occupied      -1 unknown       0 free
  double indx = (floor((Xp[1] - Xstarty) / resolution) * width) + (floor((Xp[0] - Xstartx) / resolution));
  if(int(indx) >= Data.size() || int(indx) < 0)
    return -1;
  else
    return Data[int(indx)];
}

// ObstacleFree function-------------------------------------

char ObstacleFree(std::vector<double> xnear, std::vector<double> &xnew, nav_msgs::msg::OccupancyGrid mapsub)
{
  double rez = double(mapsub.info.resolution) * .2;
  double stepz = int(ceil(norm(xnew, xnear)) / rez);
  std::vector<double> xi = xnear;
  char obs = 0;
  char unk = 0;

  geometry_msgs::msg::Point p;
  for (int c = 0; c < stepz; c++)
  {
    xi = Steer(xi, xnew, rez);

    if (grid_value(mapsub, xi) == 100)
    {
      obs = 1;
    }

    if (grid_value(mapsub, xi) == -1)
    {
      unk = 1;
      break;
    }
  }
  char out = 0;
  xnew = xi;
  if (unk == 1)
  {
    out = -1;
  }

  if (obs == 1)
  {
    out = 0;
  }

  if (obs != 1 && unk != 1)
  {
    out = 1;
  }

  return out;
}

int index_of_point(const nav_msgs::msg::OccupancyGrid &mapData, const std::vector<double> &point)
{
  auto resolution = mapData.info.resolution;
  auto Xstartx = mapData.info.origin.position.x;
  auto Xstarty = mapData.info.origin.position.y;
  auto width = mapData.info.width;
  auto Data = mapData.data;
  auto index = (int)((floor((point[1] - Xstarty) / resolution) *
                      width) +
                     (floor((point[0] - Xstartx) / resolution)));
  return index;
}

std::vector<double> point_of_index(const nav_msgs::msg::OccupancyGrid &mapData, const int &i)
{
  auto y = mapData.info.origin.position.y +
           (i / mapData.info.width) * mapData.info.resolution;
  auto x = mapData.info.origin.position.x +
           (i - (i / mapData.info.width) * (mapData.info.width)) * mapData.info.resolution;
  return {x, y};
}

double norm(const std::vector<double> &x1, const std::vector<double> &x2)
{
  return pow((pow((x2[0] - x1[0]), 2) + pow((x2[1] - x1[1]), 2)), 0.5);
}

double information_gain(const nav_msgs::msg::OccupancyGrid &mapData, const std::vector<double> &point, const double &r)
{
  auto infoGain = 0.0;
  auto index = index_of_point(mapData, point);
  auto r_region = (int)(r / mapData.info.resolution);
  auto init_index = index - r_region * (mapData.info.width + 1);
  for (int n = 0; n < 2 * r_region + 1; n++)
  {
    auto start = n * mapData.info.width + init_index;
    auto end = start + 2 * r_region;
    auto limit = ((start / mapData.info.width) + 2) * mapData.info.width;
    for (int i = start; i <= end; i++)
    {
      if ((i >= 0) && (i < limit) && (i < mapData.data.size()))
      {
        if ((mapData.data[i] == -1) && (norm(point, point_of_index(mapData, i)) <= r))
        {
          infoGain += 1;
        }
      }
    }
  }
  return infoGain * (mapData.info.resolution * mapData.info.resolution);
}

int discount(const nav_msgs::msg::OccupancyGrid &mapData,
             const std::vector<double> &assigned_pt,
             const std::vector<double> &frontier, const double &r)
{
  int index = index_of_point(mapData, assigned_pt);
  int r_region = (int)(r / mapData.info.resolution);
  int init_index = index - r_region * (mapData.info.width + 1);
  int discount = 0;
  for (int n = 0; n <= 2 * r_region; n++)
  {
    int start = n * mapData.info.width + init_index;
    int end = start + 2 * r_region;
    int limit = ((start / mapData.info.width) + 2) * mapData.info.width;
    for (int i = start; i <= end; i++)
    {
      if (i >= 0 && i < limit && i < mapData.data.size())
      {
        if (mapData.data[i] == -1 && norm(point_of_index(mapData, i), frontier) <= r && norm(point_of_index(mapData, i), assigned_pt) <= r)
        {
          // this should be modified, subtract the area of a cell, not 1
          discount += 1;
        }
      }
    }
  }
  return discount;
}

double checkAround(const nav_msgs::msg::OccupancyGrid &mapData, const std::vector<double> &point, const double &r)
{
  auto valueAtound = 0.0;
  auto index = index_of_point(mapData, point);
  auto r_region = (int)(r / mapData.info.resolution);
  auto init_index = index - r_region * (mapData.info.width + 1);
  for (int n = 0; n < 2 * r_region + 1; n++)
  {
    auto start = n * mapData.info.width + init_index;
    auto end = start + 2 * r_region;
    for (int i = start; i <= end+1; i++)
    {
      if ((i >= 0) && (i < mapData.data.size()))
      {
        if ((mapData.data[i] >0))
        {
          valueAtound += mapData.data[i];
        } 
      }
    }
  }
  return valueAtound;
}