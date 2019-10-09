


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

        std::cout << "Start record" << std::endl;
        string input;

        while (!bExit) {
            cin >> input;
            if (input == "recorddata") {
                hActuator->subscribeStartRecordEFF();
            }
            if (input == "writedata") {
                hActuator->subscribeStartWriteEFF();

            }
            if (input == "exit")
                break;

        }

        return 0;
    


}
