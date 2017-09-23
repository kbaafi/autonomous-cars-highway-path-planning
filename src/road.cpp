#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "road.h"
#include "lane.h"
#include "vehicle.h"

using namespace std;

Road::Road(int num_lanes){
  for(int i=0;i<num_lanes;i++){
    Lane l(i);
    _lanes.push_back(l);
  }
  return;
}

Road::~Road(){}

 void Road::add_vehicle(Vehicle v, int lane_id){
   Vehicle lane_vehicle;
   lane_vehicle = v;

   _lanes[lane_id].add_vehicle(lane_vehicle);

   return;
 }

 int Road::check_feasibility(){

 }
