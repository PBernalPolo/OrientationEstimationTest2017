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
 * File:   MEKFcRV.h
 * Author: P.Bernal-Polo
 * 
 * Created on February 19, 2017, 2:05 PM
 */


#ifndef MEKFCRV_H
#define MEKFCRV_H

// pi definition
#define PI 3.14159265358979323846


#include "MEKF.h"


// MEKFcRV: class implementing the Rotation Vector chart for the MEKF class
//   
//   This class implements the functions needed to perform the update of
//   the quaternion (q) and the angular velocity (w) using the
//   Manifold Extended Kalman Filter in the Rotation Vector chart
class MEKFcRV: public MEKF {
public:
  // PUBLIC METHODS
  MEKFcRV();
  MEKFcRV(const MEKFcRV& orig);
  virtual ~MEKFcRV();
  
  
private:
  // PRIVATE METHODS
  // makes a shallow copy of the MEKFcRV object
  OrientationEstimator* copy() override;
  // defines the map from the chart points, to the manifold points (through the delta quaternion)
  void fC2M( double* dx , double* delta ) override;
  // this function defines the transformation on the covariance matrix
  // when it is redefined from the chart centered in q quaternion, to the
  // chart centered in p quaternion, being them related by  p = q * delta
  void chartUpdateMatrix( double* delta , double* G ) override;
  
};

#endif /* MEKFCRV_H */

