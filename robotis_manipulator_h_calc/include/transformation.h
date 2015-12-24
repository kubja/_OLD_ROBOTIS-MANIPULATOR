/*
 * transformation.h
 *
 *  Created on: Sep 9, 2015
 *      Author: Changhyun Sung
 */

#ifndef TRANSFORMATION_H_
#define TRANSFORMATION_H_

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

// Transformation

Eigen::MatrixXd 	px( double s ) , py( double s ) , pz( double s );
Eigen::MatrixXd 	Rx( double s ) , Ry( double s ) , Rz( double s );
Eigen::MatrixXd 	ax( int s ) , ay(int s), az(int s);
Eigen::MatrixXd 	cxyz( double sx , double sy, double sz);

// Rotation

Eigen::MatrixXd 	Rotation2RPY( Eigen::MatrixXd R );
Eigen::MatrixXd 	RPY2Rotation( double Roll , double Pitch , double Yaw );

Eigen::Quaterniond 	RPY2Quaternion( double Roll , double Pitch , double Yaw );
Eigen::Quaterniond 	Rotation2Quaternion( Eigen::MatrixXd R );

Eigen::MatrixXd 	Quaternion2RPY( Eigen::Quaterniond QR );
Eigen::MatrixXd 	Quaternion2Rotation( Eigen::Quaterniond QR );

Eigen::MatrixXd 	R_plus( double Roll, double Pitch, double Yaw );
double 				sign( double x );

#endif /* TRANSFORMATION_H_ */
