#ifndef PLANNER_H
#define PLANNER_H
#include <vector>
#include <iostream>
#include "vehicle.h"

using namespace std;
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

#endif // PLANNER_H
