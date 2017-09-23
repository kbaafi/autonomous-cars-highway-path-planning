#include "vehicle.h"
#include <vector>

using namespace std;

Vehicle::Vehicle(){}

Vehicle::~Vehicle(){}

Vehicle::Vehicle(int id, float x,float y, float vx, float vy, float s, float d){
  _id = id;
  _x  = x;
  _y = y;
  _vx = vx;
  _vy = vy;
  _s = s;
  _d = d;
}

Vehicle::Vehicle( const Vehicle &obj){
  _id  = obj._id;
  _x = obj._x;
  _y = obj._y;
  _vx = obj._vx;
  _vy = obj._vy;
  _s = obj._s;
  _d = obj._d;
}
