#include <vector>
#include "road.h"
class EgoCar{
public:
  double _x;
  double _y;
  double _v;
  double _s;
  double _d;
  double _yaw;
  double _max_velocity;
  double _target_velocity;

  double _end_path_s;
  double _end_path_d;

  int _lane;
  int _candidate_best_lane;
  int _change_lane_threshold;
  int _change_lane_count;
  int _desired_lane;

  bool _is_too_close;
  double _too_close_vel;
  double _too_close_distance;
  double _emergency_too_close;

  EgoCar();
  EgoCar(double max_speed,int lane, int change_lane_threshold);
  virtual ~EgoCar();


  void drive(Road &road);
  void sense_vehicle(Road &road,Vehicle vehicle, int lane);



private:
  double distance_cost(double weight, double closest_distance, int closest_car_idx, double max_obs_distance);
  double velocity_cost_ahead(double weight, double velocity,int closest_car_idx);
  double lane_change_cost(double weight,int num_lanes, int destination_lane);
  double s_distance(double road_car_s);
  double velocity_magnitude(Vehicle v);
  void increment_change_lane_vote();
  std::vector<double> rank_lanes(Road road);
  void change_lane(int laneid);
  bool is_collision_free(Lane curr_lane, Lane target_lane, double front_radius, double back_radius, double baseline);
  void process_road_stats(Road &road, double baseline);

};
