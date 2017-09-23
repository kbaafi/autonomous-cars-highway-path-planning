#ifndef LANE_H
#define LANE_H

#include "vehicle.h"
#include <vector>


using namespace std;

class Lane{
public:
  int _id;
  vector<Vehicle> _vehicles;

  Lane(int id);
  virtual ~Lane();
  void add_vehicle(Vehicle v);
  void clear_vehicles();

  int _closest_car_ahead_id;
  int _closest_car_ahead_baseline_id;
  int _closest_car_behind_id;
  int _closest_car_id;

  double _closest_car_ahead_distance;
  double _closest_car_ahead_baseline_distance;
  double _closest_car_behind_distance;
  double _closest_car_distance;
  

  double _closest_car_ahead_vel;
  double _closest_car_ahead_baseline_vel;
  double _closest_car_behind_vel;
  double _closest_car_vel;
};

#endif
