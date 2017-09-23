#include <vector>
#include "egocar.h"
#include "road.h"
#include "vehicle.h"

EgoCar::EgoCar(){

}
EgoCar::EgoCar(double max_speed,int lane, int change_lane_threshold){
  _max_velocity = max_speed;
  _target_velocity = 0;
  _d = (double)(4*lane+1);
  _change_lane_threshold = change_lane_threshold;
  _lane = lane;
  _candidate_best_lane = lane;
  _change_lane_count = 0;
}

EgoCar::~EgoCar(){

}

double EgoCar::s_distance(double road_car_s){
  return fabs(_s-road_car_s);
}


double EgoCar::velocity_magnitude(Vehicle v){
  return sqrt((v._vx*v._vx)+(v._vy*v._vy));
}

/**
  rank_lanes(Road road)

  returns the ranking of each lane according to how fast it can be driven in
  Args: road: the road on which the car is driving
  Returns: vector of doubles indexed by the lane id containing the ranking for each lane
*/
std::vector<double> EgoCar::rank_lanes(Road road){
  std::vector<double> ranks;
  ranks.reserve(road._lanes.size());

  double max_obs_distance = 180;

  double weight_distance =2;//20
  double weight_vel_ahead = 5;
  double weight_lane_change = 1;

  for(auto lane:road._lanes){
    double cost;

    double dist_cost = distance_cost(weight_distance,lane._closest_car_ahead_baseline_distance,lane._closest_car_ahead_baseline_id,max_obs_distance);

    double vel_cost_ahead = velocity_cost_ahead(weight_vel_ahead,lane._closest_car_ahead_baseline_vel,lane._closest_car_ahead_baseline_id);

    double lc_cost = 0.2*lane_change_cost(weight_lane_change,road._lanes.size(),lane._id) ;

    double total_cost = dist_cost+vel_cost_ahead+lc_cost;

    ranks[lane._id] = total_cost;
  }

  return ranks;
}

/*
  sense_vehicle()
  creates a representation of the car on a lane in the road
  Args:
    road: the road on which the car is driving
    vehicle: detected vehicle
    lane: lane of the vehicle
*/
void EgoCar::sense_vehicle(Road &road,Vehicle vehicle, int lane){
  road.add_vehicle(vehicle,lane);
  /*if(lane == _lane){

    if((vehicle._s > _s) && ((vehicle._s - _s) < 30)){
      _is_too_close = true;
      _too_close_vel = velocity_magnitude(vehicle);
      _too_close_distance = fabs(_s - vehicle._s);
    }
  }*/
  return;
}

/*
  process_road_stats(Road road)
  retrieves needed statistics for each lane
  Args:
    road:  the road on which the car is driving
*/
void EgoCar::process_road_stats(Road &road, double baseline){
  double baseline_s = _s+baseline;
  for (auto & lane:road._lanes){
    int lid  = lane._id;
    for (Vehicle v:lane._vehicles){
      // statistics for vehicles ahead
      if(v._s > _s){
        double dist = fabs(v._s - _s);

        if(lane._closest_car_ahead_distance>dist){
          lane._closest_car_ahead_id = v._id;
          lane._closest_car_ahead_distance = dist;
          lane._closest_car_ahead_vel = velocity_magnitude(v);

          if(lane._id == _lane && lane._closest_car_ahead_distance<30){
            _is_too_close = true;
            _too_close_vel = lane._closest_car_ahead_vel;
            _too_close_distance = lane._closest_car_ahead_distance;
          }
        }
      }
      // statistics for vehicles behind
      if(v._s <=_s){
        double dist = fabs(v._s - _s);
        if(lane._closest_car_behind_distance>dist){
          lane._closest_car_behind_id = v._id;
          lane._closest_car_behind_distance = dist;
          lane._closest_car_behind_vel = velocity_magnitude(v);
        }
      }
      // statistics for strict Cartesian distance
      double diffx = _x - v._x;
      double diffy = _y - v._y;
      double cartesian = sqrt(diffx*diffx+diffy*diffy);
      if(cartesian>lane._closest_car_distance){
        lane._closest_car_distance = cartesian;
        lane._closest_car_id = v._id;
        lane._closest_car_vel = velocity_magnitude(v);
        if(cartesian<2){
          _emergency_too_close = true;
        }
      }

      // statistics according to a distance baseline
      if(v._s > baseline_s){
        double dist = fabs(v._s - baseline_s);
        if(lane._closest_car_ahead_baseline_distance>dist){
          lane._closest_car_ahead_baseline_id = v._id;
          lane._closest_car_ahead_baseline_distance = dist;
          lane._closest_car_ahead_baseline_vel = velocity_magnitude(v);
        }
      }
    }
  }
  return;
}

/*
  drive()
  controls the car as it drives on the road
  Args: the road on which the car is driving
*/
void EgoCar::drive(Road &road){

  double baseline = 20;
  double throttle_rate = .336;
  double target_vel;

  // retrive road statistics
  process_road_stats(road,-baseline);

  Lane lane = road._lanes.at(_lane);
  cout<<"baseline"<< lane._closest_car_ahead_baseline_distance<<endl;

  // handle situation where car ahead is too close
  if(_is_too_close==true){
      if(_too_close_distance<20){
        target_vel = _too_close_vel;
        throttle_rate = .560;
      }
      else {
        target_vel  = _too_close_vel+10;
      }
  }
  else{
    target_vel = _max_velocity-3; throttle_rate = .560;
  }

  // handle emergency situation where road car is likely to hit
  if(_emergency_too_close){
    if(lane._closest_car_behind_id<0){
      target_vel = 0;
      throttle_rate = 1.120;
    }
    else{
      if(target_vel>lane._closest_car_behind_vel)
        target_vel = lane._closest_car_behind_vel;
      throttle_rate = 1.120;
    }
  }

  // controls speed of car
  if(_target_velocity<target_vel){_target_velocity+=throttle_rate;}
  else {_target_velocity-=throttle_rate;}

  // get most suitable lane
  std::vector<double> ranks = rank_lanes(road);
  std::vector<double> rank_v;        //push_back hack
  rank_v.push_back(ranks[0]);rank_v.push_back(ranks[1]);rank_v.push_back(ranks[2]);

  cout<<"lane scores   "<<ranks[0]<<" "<<ranks[1] <<" "<<ranks[2]<<endl;
  int minidx = 0;
  double min_rank=9999.00;

  // best lane
  for (int i=0;i<rank_v.size();i++){
    if(rank_v[i]<min_rank){
      minidx = i;
      min_rank = ranks[i];
    }
  }

  // target next lane in direction of best lane
  // if car is not currently in best lane
  int dir = _lane - minidx;
  int desired_lane;
  int inc;
  if(dir<0){
    desired_lane = _lane+1;
  }
  else if(dir==0){desired_lane = _lane;}
  else{
    desired_lane = _lane-1;
  }

  // wait till it is suitable to change lane before executing lane change
  if(_candidate_best_lane!=minidx){
    _candidate_best_lane = minidx;

    // donot consider lane change candidates immediately after a lane change
    // see EgoCar::change_lane
    if(_change_lane_count>=0){
      _change_lane_count = 1;
    }
  }
  else{
    increment_change_lane_vote();
    if(_change_lane_threshold<=_change_lane_count){
      if(_lane!=desired_lane){
        if(is_collision_free(lane,road._lanes.at(desired_lane),15,25,baseline)){
          change_lane(desired_lane);
        }
        else{
          _change_lane_count = 0;
        }
      }
    }
  }
}


void EgoCar::change_lane(int lane){
  _d = (double) 2+4*lane;
  _lane = lane;
  _candidate_best_lane = lane;
  _change_lane_count = -100;
  _target_velocity+=1.24;
}

/*
  is_collision_free()

  performs feasibility checks on the lane change action before actual lane change
  is executed
*/
bool EgoCar::is_collision_free(Lane curr_lane, Lane target_lane, double front_radius, double back_radius, double baseline){
  bool no_collision = true;
  // feasibility check based on cars behind us
  if(curr_lane._closest_car_ahead_id>=0 && curr_lane._closest_car_ahead_distance<front_radius){
    no_collision = false;
  }

  // feasibility check based on cars ahead and slightly behind us via some -ve s baseline
  if(target_lane._closest_car_ahead_baseline_id>=0 && target_lane._closest_car_ahead_baseline_distance<(baseline+front_radius+10)){
    no_collision = false; cout<<"baseline collision"<< target_lane._closest_car_ahead_baseline_distance<<endl;
  }

  // feasibility check based on distance of car behind us
  if(target_lane._closest_car_behind_id>=0 && target_lane._closest_car_behind_distance<back_radius){
    no_collision = false;
  }

  // feasibility check based on speed of car behind us
  if(target_lane._closest_car_behind_id>=0 && target_lane._closest_car_behind_distance<back_radius){

    if((_v) <= (target_lane._closest_car_behind_vel+15)){
      no_collision = false;cout<<"WATCH OUT << "<<target_lane._closest_car_behind_vel<<" "<<_target_velocity<<endl;
    }
  }

  // changing lanes should be done at a relatively high speed
  if(_v<=30){
    no_collision = false;
  }

  return no_collision;

}

/*
  distance_cost

  calculates the cost related to how much distance there is between our car
  and the car ahead in the target lane
*/
double EgoCar::distance_cost(double weight, double closest_distance, int closest_car_idx, double max_obs_distance){
  double cost;
  if(closest_car_idx<0){
    cost = 0;
  }
  else{
    if(closest_distance>max_obs_distance){
      cost = 0;
    }
    else {
      cost = weight*(((max_obs_distance-closest_distance)/max_obs_distance));
    }
  }
  return cost;
}

/*
  velocity_cost_ahead

  calculates the cost related to the velocity of our car and the
  and the car ahead in the target lane
*/
double EgoCar::velocity_cost_ahead(double weight, double velocity,int closest_car_idx){
  double cost;

  if(closest_car_idx<0){
    cost = 0;
  }
  else{
    cost = weight*((fabs(_max_velocity - velocity)/_max_velocity));
  }
  return cost;
}

/*
  lane_change_cost

  calculates the cost related to the changing the lane the car is in
*/
double EgoCar::lane_change_cost(double weight,int num_lanes, int destination_lane){
  double cost;
  int ego_lane = (int)(_d/4)%3;

  double x = (double)abs(destination_lane - ego_lane)/(num_lanes-1);
  double weightedx = weight*x;
  cost = weightedx;

  return cost;
}

void EgoCar::increment_change_lane_vote(){
  _change_lane_count+=1;
  cout<<"increment "<<_change_lane_count<<endl<<endl;
}
