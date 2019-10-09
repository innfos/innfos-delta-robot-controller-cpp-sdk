



#include <iostream>
#include "actuatorcontroller.h"
#include <thread>
#include <signal.h>
#include <string.h>
#include <chrono>
#include "DeltaController.h"




ActuatorController * pActuator;

DeltaController *hActuator=nullptr;

InfoPrinter *InforP;
bool bExit = false;

void processSignal(int sign)
{
    ActuatorController::getInstance()->disableAllActuators();
    this_thread::sleep_for(std::chrono::milliseconds(200));
    if(hActuator)
        hActuator->bExit = true;
    bExit = true;
}


int main(int argc, char **argv) {

   

    

        InforP = new InfoPrinter("innfos-delta-robot-controller", "v1.0.0", "2019-09-29");
        InforP->addCustomInfo(
                "innfos-delta-robot-controller is for Delta Robot, the main functions of which include recording, teaching, etc.");
        InforP->showInfo();

        signal(SIGINT, processSignal);
        ActuatorController::initController();


        Actuator::ErrorsDefine ec;
        std::vector<ActuatorController::UnifiedID> idArray = ActuatorController::getInstance()->lookupActuators(ec);

        pActuator = ActuatorController::getInstance();
        pActuator->enableActuatorInBatch(idArray);

        hActuator = new DeltaController();

        std::cout << "Start calculating" << std::endl;
        string input;
        double input1;
        double input2;
        double input3;
        double input4;
        double input5;
        double input6;

        while (!bExit) {
            cin >> input;
            if (input == "calculate")
            {
                hActuator->getDeltaGeometricDim({200 * sqrt(3.0), 45 * sqrt(3.0), 163, 343, 58, -10, 55});
                std::cout<<"please input scurve parameters"<<std::endl;
                cin >> input1 >> input2 >> input3 >> input4 >> input5 >> input6;
//                hActuator->subscribeFKReadData_scurve(1000,0,0,10000,10000,100000000);
                hActuator->subscribeFKReadData_scurve(input1, input2, input3, input4, input5, input6);

            }
            //cin>>input;

            if (input == "play") {
                    hActuator->IKPlaybackCommand_Teach_scurve();
                }

            if (input == "exit")
                    break;
        }
        return 0;


   

}
