/*
 * Copyright (C) 2017 P.Bernal-Polo
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* 
 * File:   MUKF.h
 * Author: P.Bernal-Polo
 * 
 * Created on February 19, 2017, 2:05 PM
 */


#ifndef MUKF_H
#define MUKF_H


#include "config.h"
#include <math.h>  // sqrt()

#include "OrientationEstimator.h"


// MUKF: abstract class implementing the UKF for update the quaternion and the angular velocity
//   
//   This class implements most of the methods needed to perform the
//   update of the quaternion describing the orientation (q) and the
//   angular velocity (w) using the Manifold Unscented Kalman Filter. This
//   class is abstract because a chart definition is needed in order to
//   establish the full functionality
class MUKF: public OrientationEstimator {
public:
  // PUBLIC METHODS
  MUKF();
  MUKF(const MUKF& orig);
  virtual ~MUKF();
  
  // this method initializes the object when we do not have information about the state
  void initialize( double* RwIn , double* RaIn ) override;
  // gets the quaternion describing the orientation
  void get_q( double* qout ) override;
  // method used to update the state information through an IMU measurement
  void updateIMU( double* am , double* wm , double dt ) override;
  
  
protected:
  // ABSTRACT METHODS
  
  // Method: fM2C
  // defines the map from the manifold points, to the chart points
  // inputs:
  //  qm: mean quaternion of the distribution (is mapped with the origin of the chart)
  //  q: quaternion that we want to map with a point in the chart
  // outputs:
  //  e: point in the chart mapped with the q quaternion
  virtual void fM2C( double* e , double* qm , double* q ) = 0;
  
  // Method: fC2M
  // defines the map from the chart points, to the manifold points
  // inputs:
  //  qm: mean quaternion of the distribution (it is mapped with the origin of the chart)
  //  e: point of the chart that we want to map to a unit quaternion in the manifold
  // outputs:
  //  q: quaternion in the manifold mapped with the e point in the chart
  virtual void fC2M( double* q , double* qm , double* e ) = 0;
  
  
private:
  // PRIVATE VARIABLES
  // quaternion used for the last update (q1,q2,q3,q4)=(qx,qy,qz,qw)
  // (rotation that transform vectors from the sensor reference frame, to the external reference frame)
  double q0[4];
  // last updated point in the chart
  double e[3];
  // quaternion describing the orientation (q1,q2,q3,q4)=(qx,qy,qz,qw)
  // (rotation that transform vectors from the sensor reference frame, to the external reference frame)
  double q[4];
  // angular velocity (rad/s)
  double w[3];
  // covariance matrix
  double P[36];
  // covariance matrix of the angular velocity noise (rad^2/s^3)
  double Qw[9];
  // covariance matrix of the acceleration noise (g^2)
  double Qa[9];
  // covariance matrix of the angular velocity measurement noise (rad^2/s^2)
  double Rw[9];
  // covariance matrix of the acceleration measurement noise (g^2)
  double Ra[9];
  
  
  // PRIVATE STATIC METHODS
  // performs the Cholesky decomposition of a positive definite matrix ( S = L*L' )
  static void Cholesky( double* S , int n );
  // solves the system of linear equations  K*S = M  for K
  static void solve( double* S , double* M );
  // this method predicts the state given the previous state, and the time increment
  static void statePrediction( double* x , double dt );
  // this method predicts the measurement given a state
  static void IMU_MeasurementPrediction( double* y , const double* x );
  
  // OTHER PRIVATE METHODS
  void printMatrix( double* M , int n , int m );

};

#endif /* MUKF_H */

