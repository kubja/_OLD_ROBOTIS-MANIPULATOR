/*
 * transformation.cpp
 *
 *  Created on: Sep 9, 2015
 *      Author: Changhyun Sung
 */

#include "ros/ros.h"

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <stdlib.h>

#include "../../include/transformation.h"

// transformation

Eigen::MatrixXd px(double s)
{
	Eigen::MatrixXd P(3,1);

	P << s,
	     0.0,
		 0.0;

	return P;
}

Eigen::MatrixXd py(double s)
{
	Eigen::MatrixXd P(3,1);

	P << 0.0,
	     s,
		 0.0;

	return P;
}

Eigen::MatrixXd pz(double s)
{
	Eigen::MatrixXd P(3,1);

	P << 0.0,
		 0.0,
		 s;

	return P;
}

Eigen::MatrixXd Rx(double s)
{
	Eigen::MatrixXd R(3,3);

	R << 1.0, 	 0.0, 	  0.0,
	     0.0, cos(s), -sin(s),
	     0.0, sin(s),  cos(s);

    return R;
}

Eigen::MatrixXd Ry(double s)
{
	Eigen::MatrixXd R(3,3);

	R << cos(s), 0.0, sin(s),
	     0.0, 	 1.0, 	 0.0,
	    -sin(s), 0.0, cos(s);

    return R;
}

Eigen::MatrixXd Rz(double s)
{
	Eigen::MatrixXd R(3,3);

	R << cos(s), -sin(s), 0.0,
	     sin(s),  cos(s), 0.0,
		    0.0,     0.0, 1.0;

	return R;
}

Eigen::MatrixXd ax(int s)
{
	Eigen::MatrixXd A(3,1);

	A << s,
	     0.0,
		 0.0;

	return A;
}

Eigen::MatrixXd ay(int s)
{
	Eigen::MatrixXd A(3,1);

	A << 0.0,
	     s,
		 0.0;

	return A;
}

Eigen::MatrixXd az(int s)
{
	Eigen::MatrixXd A(3,1);

	A << 0.0,
	     0.0,
		 s;

	return A;
}

Eigen::MatrixXd cxyz(double sx, double sy, double sz)
{
	Eigen::MatrixXd C(3,1);

	C << sx,
	     sy,
		 sz;

	return C;
}

Eigen::MatrixXd Rotation2RPY(Eigen::MatrixXd R)
{
	Eigen::MatrixXd RPY = Eigen::MatrixXd::Zero(3,1);

	RPY.coeffRef(0,0) = atan2( R.coeff(2,1), R.coeff(2,2));
	RPY.coeffRef(1,0) = atan2( -R.coeff(2,0), sqrt( pow(R.coeff(2,1),2) + pow(R.coeff(2,2),2) ) );
	RPY.coeffRef(2,0) = atan2 (R.coeff(1,0) , R.coeff(0,0) );

	return RPY;
}

Eigen::MatrixXd RPY2Rotation(double Roll, double Pitch, double Yaw)
{
	Eigen::MatrixXd Rotation = Rz(Yaw)*Ry(Pitch)*Rx(Roll);

	return Rotation;
}

Eigen::Quaterniond RPY2Quaternion(double Roll, double Pitch, double Yaw)
{
	Eigen::MatrixXd R = RPY2Rotation(Roll, Pitch, Yaw);

	Eigen::Matrix3d R_plus;

	R_plus = R.block(0,0,3,3);

	Eigen::Quaterniond QR;

	QR = R_plus;

	return QR;
}

Eigen::Quaterniond Rotation2Quaternion(Eigen::MatrixXd R)
{
	Eigen::Matrix3d R_plus;

	R_plus = R.block(0,0,3,3);

	Eigen::Quaterniond QR;

	QR = R_plus;

	return QR;
}

Eigen::MatrixXd Quaternion2RPY(Eigen::Quaterniond QR)
{
	Eigen::MatrixXd RPY = Rotation2RPY(QR.toRotationMatrix());

	return RPY;
}

Eigen::MatrixXd Quaternion2Rotation(Eigen::Quaterniond QR)
{
	Eigen::MatrixXd Rotation = QR.toRotationMatrix();

	return Rotation;
}

Eigen::MatrixXd R_plus( double Roll, double Pitch, double Yaw )
{
	Eigen::MatrixXd R = Eigen::MatrixXd::Zero(4,4);
	R.coeffRef(3,3) = 1.0;

	Eigen::MatrixXd R_plus = Rz(Yaw) * Ry(Pitch) * Rx(Roll);

	R.block(0,0,3,3) = R_plus;

	return R;
}

double sign( double x )
{
	double sign;

	if (x < 0.0 )
		sign = -1.0;
	else if ( x > 0.0)
		sign = 1.0;
	else
		sign = 0.0;

	return sign;
}
