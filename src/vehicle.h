#ifndef VEHICLE_H
#define VEHICLE_H
#include <vector>
#include <iostream>

class Vehicle
{
public:
    Vehicle();
    double id, s, d;
    Vehicle(double id, double s, double d);
    ~Vehicle();
};

#endif // VEHICLE_H
