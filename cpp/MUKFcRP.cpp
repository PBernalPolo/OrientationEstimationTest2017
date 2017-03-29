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
 * File:   MUKFcRP.cpp
 * Author: P.Bernal-Polo
 * 
 * Created on February 19, 2017, 2:05 PM
 */


#include "MUKFcRP.h"


MUKFcRP::MUKFcRP() {
}

MUKFcRP::MUKFcRP(const MUKFcRP& orig) {
}

MUKFcRP::~MUKFcRP() {
}


// Method: copy
// makes a shallow copy of the MUKFcRP object
// inputs:
// outputs:
//  OrientationEstimator*: an OrientationEstimator pointer, pointing to the MUKFcRP object
OrientationEstimator* MUKFcRP::copy(){
  return new MUKFcRP( *this );
}


// Method: fM2C
// defines the map from the manifold points, to the chart points
// inputs:
//  qm: mean quaternion of the distribution (is mapped with the origin of the chart)
//  q: quaternion that we want to map with a point in the chart
// outputs:
//  e: point in the chart mapped with the q quaternion
void MUKFcRP::fM2C( double* e , double* qm , double* q ){
  // first we compute the delta in the manifold
  double delta[4];
  delta[0] = qm[0]*q[0] + qm[1]*q[1] + qm[2]*q[2] + qm[3]*q[3];
  delta[1] = qm[0]*q[1]  -  q[0]*qm[1]  -  qm[2]*q[3] + qm[3]*q[2];
  delta[2] = qm[0]*q[2]  -  q[0]*qm[2]  -  qm[3]*q[1] + qm[1]*q[3];
  delta[3] = qm[0]*q[3]  -  q[0]*qm[3]  -  qm[1]*q[2] + qm[2]*q[1];
  // e from the chart definition: Rodrigues Parameters
  double aux = 2.0/delta[0];
  e[0] = delta[1]*aux;
  e[1] = delta[2]*aux;
  e[2] = delta[3]*aux;
  
  return;
}


// Method: fC2M
// defines the map from the chart points, to the manifold points
// inputs:
//  qm: mean quaternion of the distribution (it is mapped with the origin of the chart)
//  e: point of the chart that we want to map to a unit quaternion in the manifold
// outputs:
//  q: quaternion in the manifold mapped with the e point in the chart
void MUKFcRP::fC2M( double* q , double* qm , double* e ){
  // delta from the chart definition: Rodrigues Parameters
  double aux = 1.0/sqrt( 4.0 + e[0]*e[0] + e[1]*e[1] + e[2]*e[2] );
  double delta[4];
  delta[0] = 2.0*aux;
  delta[1] = e[0]*aux;
  delta[2] = e[1]*aux;
  delta[3] = e[2]*aux;
  // now we update in the manifold with this delta
  q[0] = qm[0]*delta[0] - qm[1]*delta[1] - qm[2]*delta[2] - qm[3]*delta[3];
  q[1] = qm[0]*delta[1]  +  delta[0]*qm[1]  +  qm[2]*delta[3] - qm[3]*delta[2];
  q[2] = qm[0]*delta[2]  +  delta[0]*qm[2]  +  qm[3]*delta[1] - qm[1]*delta[3];
  q[3] = qm[0]*delta[3]  +  delta[0]*qm[3]  +  qm[1]*delta[2] - qm[2]*delta[1];
  
  return;
}



