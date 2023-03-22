#include "control_loop.h"

using namespace std;

//the goal is to implement a simple PID controller, 
//running on input value to reach the desired one

// Sense: read a value from keyboard
// Plan:  generate the correct input
// Act:   set the input

int main(int argc, char** argv) {

    double X_TARGET;
    double KP; // proportional gain 
    double KD; // derivative gain 
    double KI; // integrative gain 


    cout<<"Please set the proportional gain:"<<endl;
    cin>>KP; 
    cout<<"Please set the derivative gain:"<<endl;
    cin>>KD;
    cout<<"Please set the integrative gain:"<<endl;
    cin>>KI;


    // start the entire control program the first time 
    CONTROLLER Controller_Ciampi(X_TARGET,0.01,KP,KD,KI);

    while(true){
        cout<<"Please set the target value:"<<endl;
        cin>>X_TARGET;
        Controller_Ciampi.set_xdes(X_TARGET);
        // start only the control loop each target change 
        Controller_Ciampi.loop();
        // safe exit 
        if (X_TARGET==0) exit(1);
        
    }
    return 0;
}
