#include "control_loop.h"

//We can use the class constructor to set parameters
CONTROLLER::CONTROLLER(double target, double delta_time, double P, double D, double I) {
    
    //Sense: get input to change the state of our System
    set_xdes(target);
    if (delta_time>0) dt=delta_time;
    else dt=0.01; // default delta time 
    if (P>0) Kp=P;
    else Kp=1; // default KP 
    if (D>0) Kd=D;
    else Kd=0; // default KD  
    if (I>0) Ki=I;
    else Ki=0; // default KI 

    start = false; // activation function 

    system_start(); 
   
    boost:: thread loop_thread(&CONTROLLER::loop,this); 

}

void CONTROLLER:: set_xdes(double x){
    x_target=x;
    start= true; 
    // when the target is defined, start the loop 
}
//Random initial value
void CONTROLLER::system_start() {

    // define random value in [0,1[ for the initial position 
    srand((unsigned) time(NULL));
    x_measured = rand()/(RAND_MAX+1.0);
    
}

void CONTROLLER::loop() {

    double e=0.0; // error 
    double pre_e=0.0; // error at dt(t-1)
    double P_e=0.0; // error for proportional term 
    double D_e=0.0; // derivative of the error 
    double I_e=0.0; // integral of the error 
    int _time =0.0; // count of delta time 
    double PID=0.0; // PID control action 

    while (!start){
        // wait until the definition of a target 
        usleep(0.1e6);
    }

    // open two text file to write the results of control 
    ofstream f_target("TargetValue.txt",std::ofstream::app);
    ofstream f_control("PIDcontrol.txt",std::ofstream::app);
    
    // print the 
    cout << "starting point:" << x_measured << "\n target value: " << x_target << endl;
    
    while (true){

        // write on files 
        f_target << x_target << endl;
        f_control << x_measured << endl;

        // set the precedent error 
        pre_e = e;

        // calculate error 
        e = x_target - x_measured;


        if(abs(e)<0.01) {
            // stop the loop when the error is acceptable 
            cout << "success" << endl;
            break;
        
        }
        if(abs(e)-abs(pre_e)>1e5 || _time>10000) {
            // stop the loop when the function is divergent or the time is too long 
            cout << "error occurred" << endl;
            break;
        }

        // proportional term 
        P_e = e;

        // derivative term 
        D_e = (e - pre_e) / dt;

        // integrative term 
        I_e += e * dt; 

        // pid 
        PID = Kp * P_e + Kd * D_e + Ki * I_e;

        // output updated 
        x_measured += PID;

        // print the error and the simulation time 
        cout << "Error: "<<e<<"\n";
        cout << "Simulation time: "<< _time*dt<<"\n";
        usleep(0.1e-6); 
        _time++; // update the count 



    }

    f_target.close();
    f_control.close();


}


