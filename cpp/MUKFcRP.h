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
 * File:   MUKFcRP.h
 * Author: P.Bernal-Polo
 * 
 * Created on February 19, 2017, 2:05 PM
 */


#ifndef MUKFCRP_H
#define MUKFCRP_H


#include "MUKF.h"


// MUKFcRP: class implementing the Rodrigues Parameters for the MUKF class
//   
//   This class implements the functions needed to perform the update of
//   the quaternion (q) and the angular velocity (w) using the
//   Manifold Unscented Kalman Filter in the Rodrigues Parameters chart
class MUKFcRP: public MUKF {
public:
  // PUBLIC METHODS
  MUKFcRP();
  MUKFcRP(const MUKFcRP& orig);
  virtual ~MUKFcRP();
  
  
private:
  // PRIVATE METHODS
  // makes a shallow copy of the MUKFcRP object
  OrientationEstimator* copy() override;
  // defines the map from the manifold points, to the chart points
  void fM2C( double* e , double* qm , double* q ) override;
  // defines the map from the chart points, to the manifold points
  void fC2M( double* q , double* qm , double* e ) override;
  
};

#endif /* MUKFCRP_H */

