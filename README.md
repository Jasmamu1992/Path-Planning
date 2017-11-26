# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Traffic Prediction
In this step the position of all the traffic vehicles obtained from sensor fusion is extrapolated to predict their position at the time when all the previous path points of the ego car are executed. The extrapolation is done assuming the traffic is moving at constant velocity. This is done because the acceleration information is not available in the sensor fusion data


```C++
vector<Vehicle> predictions;
for (int i = 0; i < sensor_fusion.size(); i++){
   double id = (double)sensor_fusion[i][0];
   double x = sensor_fusion[i][1];
   double y = sensor_fusion[i][2];
   double vx = sensor_fusion[i][3];
   double vy = sensor_fusion[i][4];
   x += ((double)prev_size * 0.02 * vx);
   y += ((double)prev_size * 0.02 * vy);
   double theta = atan2(vy, vx);
   vector<double> sd = getFrenet(x, y, theta, map_waypoints_x, map_waypoints_y);
   double check_car_s = sd[0];
   double check_car_d = sd[1];
   Vehicle vehicle(id, check_car_s, check_car_d);
   predictions.push_back(vehicle);
}
```

### Path Planning
Planner class is implemented, which maintains the path planning state machine. Three states are taken into consideration, KL(Keep Lane), LCL(Lane Change Left), LCR(Lane Change Right).

```C+++
class Planner
{
public:
    Planner();
    virtual ~Planner();
    vector<string> states = {"KL", "LCL", "LCR"};
    string state;
    void planned_state(vector<Vehicle> predictions, int lane, double car_s);
    void realize_state(int &lane);
};
```
Based on the predictions of the traffic vechiles, one of the three states is switched. The state machine is called only if the ego vehicle is less than 25m away from the traffic ahead in the same lane.

#### States

KL - If there is traffic in the adjacent lanes whose S value is less than 25m away from the S value of the ego vehicle, the ego vehicle stays in its current lane and follows the vehicle in the front by changing its speed to match the speed of the traffic vehicle ahead of it.
LCL, LCR - If there is no traffic in the adjacent lanes whose S value is less than 25m away from the S value of the ego vehicle, the ego vehicle will try to change lane which has the most free space. The Ego vehilce will try to maintain velocity close to speed limit.

#### Trajectory generator
Spline library is used to generate trajectories in such a way that the maximum jerk and acceleration are never exceeded. The implementation of the trajectory generator is similar to the one explained in the class. Points are picked from the intended path and spline library is used to interpolate these points. The interpolation obrained from the spline library is carefully samplled in such a way that the maximum jerk and acceleration is never exceeded.

Interpolating using spline library
```C++
tk::spline s;
s.set_points(ptsx, ptsy);
```
Sampling the spline interpolation
```C++
double target_x = 30;
double target_y = s(target_x);
double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
double x_add_on = 0;

for (int i = 0; i <= 50-previous_path_x.size(); i++){
   double N = (target_dist / (0.02 * ref_vel/2.24));
   double x_point = x_add_on + (target_x)/N;
   double y_point = s(x_point);
   x_add_on = x_point;
   double x_ref = x_point;
   double y_ref = y_point;
   x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
   y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
   x_point += ref_x;
   y_point += ref_y;
   next_x_vals.push_back(x_point);
   next_y_vals.push_back(y_point);
}
```


