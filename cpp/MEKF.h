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
 * File:   MEKF.h
 * Author: P.Bernal-Polo
 * 
 * Created on February 19, 2017, 2:05 PM
 */


#ifndef MEKF_H
#define MEKF_H


#include "config.h"
#include <math.h>  // sqrt()

#include "OrientationEstimator.h"


// MEKF: abstract class implementing the MEKF for updating the quaternion and the angular velocity
//   
//   This class implements most of the methods needed to perform the
//   update of the quaternion describing the orientation (q) and the
//   angular velocity (w) using the Manifold Extended Kalman Filter. This 
//   class is abstract because a chart definition is needed in order to
//   establish the full functionality
class MEKF: public OrientationEstimator {
public:
  // PUBLIC METHODS
  MEKF();
  MEKF(const MEKF& orig);
  virtual ~MEKF();
  
  // this method initializes the object when we do not have information about the state
  void initialize( double* RwIn , double* RaIn ) override;
  // gets the quaternion describing the orientation
  void get_q( double* qout ) override;
  // method used to update the state information through an IMU measurement
  void updateIMU( double* am , double* wm , double dt ) override;
  
  
protected:
  // PROTECTED ABSTRACT METHODS
  
  // Method: fC2M
  // defines the map from the chart points, to the manifold points (through the delta quaternion)
  // inputs:
  //  e: point of the Euclidean space that we want to map to a unit quaternion
  // outputs:
  //  delta: quaternion mapped with the e point
  virtual void fC2M( double* dx , double* delta ) = 0;
  
  // Method: chartUpdateMatrix
  // this function defines the transformation on the covariance matrix
  // when it is redefined from the chart centered in q quaternion, to the
  // chart centered in p quaternion, being them related by  p = q * delta
  // inputs:
  //  delta: quaternion used to update the quaternion estimation
  // outputs:
  //  G: transformation matrix to update the covariance matrix
  virtual void chartUpdateMatrix( double* delta , double* G ) = 0;
  
  
private:
  // PRIVATE VARIABLES
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
  static void Cholesky( double* S );
  // solves the system of linear equations  K*S = M  for K
  static void solve( double* S , double* M );
  
protected:
  void printMatrix( double* M , int n , int m );

};

#endif /* MEKF_H */

