#ifndef SPEED_PROFILE_H
#define SPEED_PROFILE_H

#define pi 3.1416

class speed_profile
{
private:
	double s;
	double f;//最大速度
	double fs;//初速度
	double fe;//末速度
	double A;//最大加速度
	double D;//最小减速度
	double J;//最大加加速度

	double v1;
	double a1;
	double s1;

	double v2;
	double a2;
	double s2;

	double v3;
	double a3;
	double s3;

	double v4;
	double a4;
	double s4;

	double v5;
	double a5;
	double s5;

	double v6;
	double a6;
	double s6;

	double v7;
	double a7;
	double s7;

public:
	double T[7];
	double tacc;
	double tdec;
	double tall;
   /**
    * @brief Set basic parameters
    * @param s position
    * @param f max velocity
    * @param fs initial velocity
    * @param fe final velocity
    * @param A  max acceleration
    * @param D  min acceleration
    * @param J  max jerk
   */
	void set_parameters(double s,double f,double fs,double fe,double A,double D,double J);
	/**
     * @brief Calculate S curve segmentation time
    */
	void calculate_jerk_limit_profile_time();
	void time_rounding();
	void calculate_flow_parameters();
	void jerk_limit_profile();
	void jerk_limit_profile_rounding();
	/**
    * @brief calculate S curve
    * @brief t time
    * @brief PAVJ[0] velocity PAVJ[1] position
    */

	void calculate_current_speed(double t,double PAVJ[]);
	void recalculate_profile(double time[]);
	void print_profile();
};

#endif
