#ifndef VEHICLE_H
#define VEHICLE_H

using namespace std;

class Vehicle{
public:
  Vehicle();
  Vehicle(int id, float x,float y, float vx, float vy, float s, float d);
  Vehicle( const Vehicle &obj);
  virtual ~Vehicle();

  int _id;
  float _x;
  float _y;
  float _vx;
  float _vy;
  float _s;
  float _d;
};
#endif
