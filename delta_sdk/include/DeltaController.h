

#ifndef DELTA_DELTA_H
#define DELTA_DELTA_H
#include <iostream>
#include "actuatorcontroller.h"
#include <thread>
#include <signal.h>
#include <string.h>
#include <vector>
#include <chrono>
#include <fstream>
#include <sstream>
#include "DeltaKinematics.h"
#include "speed_profile.h"
#include "infoPrinter.h"


class DeltaController{

public:
    DeltaController();
    ~DeltaController();
    /**
     * @brief record delta interpolation point
     */
    void subscribeStartRecordEFF();
    /**
     * @brief save delta interpolation point.
     */
    void subscribeStartWriteEFF();
    /**
     * @brief calculate delta S curve
     * @param s_vmax max velocity
     * @param s_vinit init velocity
     * @param s_vend end velocity
     * @param s_amax max acceleration
     * @param s_amin min acceleration
     * @param s_jerk max jerk
     */
    void subscribeFKReadData_scurve(double s_vmax,double s_vinit ,double s_vend,double s_amax,double s_amin,double s_jerk);
    /**
     * @brief play delta S curve.
     */
    void IKPlaybackCommand_Teach_scurve();
    /**
     * @brief read file data.
     * @param parameter_path file name
     */
    std::vector<std::vector<double> > readParameter(std::string parameter_path);
    /**
     * @brief get delta geometric basic parameters .
     * @param test_robot_dim geometric basic parameters
     */
    void getDeltaGeometricDim(DeltaKinematics<double>::DeltaGeometricDim test_robot_dim);


private:

    double  res;
    std::vector<std::vector<double> > recorded_positions_teach;
    std::vector<std::vector<double> > FKReadDataTest_scurve;
    std::vector<std::vector<double> > recorded_FKdata_scurve;
    std::vector<std::vector<double> > sCurve0;
    std::vector<std::vector<double> > IKReadDataTest_sCurve;
    std::vector<std::vector<double> > recorded_IKdata_sCurve;
    double dataPositionJoint11_scurve;
    double dataPositionJoint22_scurve;
    double dataPositionJoint33_scurve;
    ActuatorController * pActuator;
    int playProcess;
    std::string parameter_path;

    double d_s_vmax;//max velocity
    double d_s_vinit;//init velocity
    double d_s_vend;//end velocity
    double d_s_amax;//max acceleration
    double d_s_amin;//min acceleration
    double d_s_jerk;//max jerk
    DeltaKinematics<double>::DeltaGeometricDim test_robot_dim1;//geometric basic parameters

public:
    bool bExit;


};






#endif //DELTA_DELTA_H
