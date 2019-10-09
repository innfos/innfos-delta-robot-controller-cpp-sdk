#ifndef __DELTAKINEMATICS_H__
#define __DELTAKINEMATICS_H__
#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>

/**
 * @brief PI value
 */
#define M_PI 3.14159265358979323846

/** 
 * @brief Delta Robot kinematics class.
 */

template <typename RealDataType>
class DeltaKinematics
{
  public:
	/** 
	* @brief Structure that represents basic vector.
	*/
	struct DeltaVector
	{
		RealDataType x;	// cartesian position in base reference frame
		RealDataType y;	// cartesian position in base reference frame
		RealDataType z;	// cartesian position in base reference frame
		RealDataType phi1; // joint 1 angle [deg] ( negative above the base platform )
		RealDataType phi2; // joint 2 angle [deg] ( negative above the base platform )
		RealDataType phi3; // joint 3 angle [deg] ( negative above the base platform )

		/**
		 * @brief Prints values of all the cartesian position and joints variables.
		 */
		void Print();

		/**
		 * @brief Sets all position parameters to zero;
		 */
		void Clear();
	};

	/** 
	* @brief Struct that represents element of trajectory.
	*
	* Trajectory is a time history of position, velocity and acceleration for each DOF.
	*/
	struct DeltaTrajectory
	{
		DeltaVector pos;   // position
		DeltaVector vel;   // velocity
		DeltaVector accel; // acceleration
	};

	/** 
	 * @brief Delta Robot geometric basic parameters.
	 */
	struct DeltaGeometricDim
	{
		RealDataType sb;					  // base equilateral triangle side [ mm ]
		RealDataType sp;					  // platform equilateral triangle side [ mm ]
		RealDataType L;						  // upper legs length [ mm ]
		RealDataType l;						  // lower legs parallelogram length [ mm ]
		RealDataType h;						  // lower legs prallelogram width [ mm ]
		RealDataType max_neg_angle;			  // max negative angle that each arm can achive ( knee above the fixed-base plane ) [ deg ]
		RealDataType min_parallelogram_angle; // the limitation introduced by universal joints [ deg ]
	};

	/** 
	 * @brief DeltaKinematics constructor.
	 *
	 * @param dim a 7 basic parameters.
	 */
	DeltaKinematics(DeltaGeometricDim dim);

	/**
	 * @brief Calculates inverse kinematics.
	 *
	 * @param v a pointer to the matrix of cartesian positions and joints vectors, which only joints are changed.
	 * @param num a number of vectors in the matrix
	 * @return 1 if 1 if there was unreachable position, 0 if success
	 */
	int CalculateIpk(DeltaVector *v, int num);

	/**
	 * @brief Calculates forward  kinematics.
	 *
	 * @param v a pointer to the matrix of cartesian positions and joints vectors, which only cartesian positions are changed.
	 * @param num a number of vectors in the matrix
	 * @return 1 if there was unreachable position or if singularity happend, 0 if success
	 */
	int CalculateFpk(DeltaVector *v, int num);




  private:
	RealDataType _sb;						   // base equilateral triangle side [ mm ]
	RealDataType _sp;						   // platform equilateral triangle side [ mm ]
	RealDataType _Ll;						   // upper legs length [ mm ]
	RealDataType _l;						   // lower legs parallelogram length [ mm ]
	RealDataType _h;						   // lower legs prallelogram width [ mm ]
	RealDataType _wb;						   // planar distance from base reference frame to near base side [ mm ]
	RealDataType _ub;						   // planar distance from base reference frame to a base vertex [ mm ]
	RealDataType _wp;						   // planar distance from platform reference frame to near platform side [ mm ]
	RealDataType _up;						   // planar distance from platform reference frame to a platform vertex [ mm ]
	RealDataType _Pp1[3];					   // platorm-fixed U-joint virtual connection in the local platform frame
	RealDataType _Pp2[3];					   // platorm-fixed U-joint virtual connection in the local platform frame
	RealDataType _Pp3[3];					   // platorm-fixed U-joint virtual connection in the local platform frame
	RealDataType _B1[3];					   // fixed-base revolute joint point
	RealDataType _B2[3];					   // fixed-base revolute joint point
	RealDataType _B3[3];					   // fixed-base revolute joint point
	RealDataType _b1[3];					   // fixed-base vertex
	RealDataType _b2[3];					   // fixed-base vertex
	RealDataType _b3[3];					   // fixed-base vertex
	RealDataType _A1[3];					   // first knee point
	RealDataType _A2[3];					   // second knee point
	RealDataType _A3[3];					   // third knee point
	RealDataType _max_neg_angle;			   // max negative angle that each arm can achive ( knee above the fixed-base plane ) [ deg ]
	RealDataType _min_parallelogram_angle;	 ///< the limitation introduced by universal joints [ deg ]
	static const RealDataType _SQRT3;		   ///< sqrt(3)
	static const RealDataType _HSQRT3;		   ///< sqrt(3)/2
	static const RealDataType _ROTZ120[3][3];  ///< basic rotation matrix for z axis and 120 degree
	static const RealDataType _MROTZ120[3][3]; ///< basic roation matrix for z axis and -120 degree
	static const RealDataType _DEG2RAD_FACTOR; ///< basicly equals M_PI/180
	static const RealDataType _RAD2DEG_FACTOR; ///< basilcy equals 180/M_PI

	/**
	 * @brief Initialise the class members.
	 */
	void Initialise();

	/**
	 * @brief Performs vector rotation by given rotation matrix.
	 * 
	 * @param point a position vector
	 * @param matrix a rotation matrix
	 */
	void RotateByMatrix(RealDataType *point, const RealDataType (*matrix)[3]);

	/**
	 * @brief Calculates a joint angle .
	 *
	 * @param B a rotated base joint point
	 * @param P a rotated platform joint point
	 * @param phi an angle that will be calculated
	 * @return 0 if success 1 if error
	 */
	int CalculateAngle(const RealDataType *B, const RealDataType *P, RealDataType *phi);

	/**
	 * @brief Calculates forward  kinematics method:three spheres intersection algorithm ( different z hights ).
	 *
	 * @param v a position vector
	 * @return 0 if success 1 if error
	 */
	int ThreeSpheresIntersectionA(DeltaVector *v);

	/**
	 * @brief Calculates forward  kinematics method: Three spheres intersection algorithm ( the same z hights ).
	 *
	 * @param v a position vector
	 * @return 0 if success 1 if error
	 */
	int ThreeSpheresIntersectionB(DeltaVector *v);
};
#endif /* __DELTAKINEMATICS_H__ */
