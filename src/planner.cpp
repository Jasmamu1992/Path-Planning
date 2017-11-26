
#include "planner.h"
#include <iostream>
#include <cmath>

using namespace std;

Planner::Planner()
{

}

Planner::~Planner(){

}

void Planner::planned_state(vector<Vehicle> predictions, int lane, double car_s){
    state = "KL";
    int lc_thresh = 25;
    double closest_inlane = 1000000000;
    double closest_leftlane = 1000000000;
    double closest_rightlane = 1000000000;
    for(int i = 0; i < predictions.size(); i++){
        double id = predictions[i].id;
        double s  = predictions[i].s;
        double d  = predictions[i].d;

        if((d < (2+4*lane+2)) && (d > (2+4*lane-2))){
            if((s > car_s) && ((s-car_s) < closest_inlane)){
                closest_inlane = (s-car_s);
            }
        }
        if (lane>0){
            if((d < (2+4*(lane-1)+2)) && (d > (2+4*(lane-1)-2))){
                if(abs(s-car_s) < closest_leftlane){
                    closest_leftlane = abs(s-car_s);
                }
            }
        }
        if (lane<2){
            if((d < (2+4*(lane+1)+2)) && (d > (2+4*(lane+1)-2))){
                if(abs(s-car_s) < closest_rightlane){
                    closest_rightlane = abs(s-car_s);
                }
            }
        }
    }
    cout << closest_inlane << "," << closest_leftlane <<
            "," << closest_rightlane << "," << lane << endl;
    if(lane == 1){
        if ((closest_leftlane > lc_thresh) && (closest_leftlane > closest_inlane) && (closest_leftlane >= closest_rightlane) && (lane > 0)){
            state = "LCL";
        }
        else if ((closest_rightlane > lc_thresh) && (closest_rightlane > closest_inlane) && (closest_rightlane >= closest_leftlane) && (lane < 2)){
            state = "LCR";
        }
    }
    else if (lane == 0){
        if ((closest_rightlane > lc_thresh) && (closest_rightlane > closest_inlane))
            state = "LCR";
    }
    else if (lane == 2){
        if ((closest_leftlane > lc_thresh) && (closest_leftlane > closest_inlane))
            state = "LCL";
    }
}

void Planner::realize_state(int &lane){
    if (state=="KL")
        return;
    else if (state=="LCL")
        lane = lane - 1;
    else if (state=="LCR")
        lane += 1;
    return;
}

