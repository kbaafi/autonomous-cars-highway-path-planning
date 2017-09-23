#ifndef ROAD_H
#define ROAD_H

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
#include "lane.h"
#include "vehicle.h"

using namespace std;

class Road {
public:
    vector<Lane> _lanes;
    map<int, double> _lane_rankings;

    Road(int num_lanes);
    virtual ~Road();

    void add_vehicle(Vehicle v, int lane_id);
    
    int check_feasibility();

};

#endif
