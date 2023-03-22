#include <iostream>
#include <fstream>
#include "boost/thread.hpp"


using namespace std;

class CONTROLLER {
    public:
        CONTROLLER(double target, double delta_time, double P, double D, double I);
        
        void loop();                //Main loop function        
        void system_start();       //start the system
        void set_xdes(double x);   //member to set the desired value

    private:
        double x_target;
        double dt; 
        double Kp; 
        double Kd;
        double Ki;
        double x_measured;
        bool start;
        
        
};
