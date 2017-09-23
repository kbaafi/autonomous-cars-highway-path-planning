#include "lane.h"
#include "vehicle.h"
#include <iostream>
#include <vector>

using namespace std;

Lane::Lane(int id){
  _id = id;
  _closest_car_ahead_id = -1;
  _closest_car_behind_id= -1;
  _closest_car_ahead_baseline_id = -1;
  _closest_car_id= -1;

   _closest_car_ahead_distance = 9999;
   _closest_car_behind_distance = 9999;
   _closest_car_distance= 9999;
   _closest_car_ahead_baseline_distance = 9999;

   _closest_car_ahead_vel= 9999;
   _closest_car_behind_vel= 9999;
   _closest_car_vel= 9999;
   _closest_car_ahead_baseline_vel = 9999;
}

Lane::~Lane(){
  clear_vehicles();
}

void Lane::add_vehicle(Vehicle v){
  Vehicle v_elem = v;
  _vehicles.push_back(v_elem);
  return;
}

void Lane::clear_vehicles(){
  _vehicles.clear();
  return;
}
