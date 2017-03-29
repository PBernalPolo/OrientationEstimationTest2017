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
 * File:   MEKF.cpp
 * Author: P.Bernal-Polo
 * 
 * Created on February 19, 2017, 2:05 PM
 */


#include <iostream>
#include <iomanip>

#include "MEKF.h"


MEKF::MEKF() {
  this->q[0] = 1.0;   this->q[1] = 0.0;   this->q[2] = 0.0;   this->q[3] = 0.0;
  this->w[0] = 0.0;   this->w[1] = 0.0;   this->w[2] = 0.0;
  
  for(int k=0; k<36; k++) this->P[k] = 0.0;
  for(int k=0; k<36; k+=7) this->P[k] = 1.0e2;
  this->P[2+2*6] = 1.0e-16;
  
  for(int k=0; k<9; k++){
    this->Qw[k] = 0.0;
    this->Qa[k] = 0.0;
    this->Rw[k] = 0.0;
    this->Ra[k] = 0.0;
  }
  for(int k=0; k<9; k+=4){
    this->Qw[k] = 1.0e0;
    this->Qa[k] = 1.0e-2;
    this->Rw[k] = 1.0e-4;
    this->Ra[k] = 1.0e-4;
  }
  
}

MEKF::MEKF(const MEKF& orig) {
}

MEKF::~MEKF() {
}


// Method: initialize
// this method initializes the object when we do not have information about the state
// inputs:
//  RwIn: covariance matrix of the angular velocity measurement
//  RaIn: covariance matrix of the acceleration measurement
// outputs:
void MEKF::initialize( double* RwIn , double* RaIn ){
  q[0] = 1.0;    q[1] = 0.0;    q[2] = 0.0;    q[3] = 0.0;
  w[0] = 0.0;    w[1] = 0.0;    w[2] = 0.0;
  
  for(int k=0; k<36; k++) P[k] = 0.0;
  for(int k=0; k<36; k+=7) P[k] = 1.0e2;
  
  for(int k=0; k<9; k++){
    Qw[k] = 0.0;
    Qa[k] = 0.0;
  }
  for(int k=0; k<9; k+=4){
    Qw[k] = 1.0e0;
    Qa[k] = 1.0e-2;
  }
  
  for(int k=0; k<9; k++){
    Rw[k] = RwIn[k];
    Ra[k] = RaIn[k];
  }
  
  return;
}


// Method: get_q
// gets the quaternion describing the orientation
// (rotation that transform vectors from the sensor reference frame, to the external reference frame)
// (in case your quaternion transforms vectors from the external reference frame to the sensor reference frame, this function should return q^*)
// inputs:
// outputs:
//  q: quaternion describing the orientation (q1,q2,q3,q4)=(qx,qy,qz,qw)
void MEKF::get_q( double* qout ){
  qout[0] = q[0];
  qout[1] = q[1];
  qout[2] = q[2];
  qout[3] = q[3];
  
  return;
}


// Method: updateIMU
// method used to update the state information through an IMU measurement
// inputs:
//  am: measured acceleration (g)
//  wm: measured angular velocity (rad/s)
//  dt: time step from the last update (s)
// outputs:
void MEKF::updateIMU(double* am, double* wm, double dt){
  // we compute the state prediction
  double wnorm = sqrt( w[0]*w[0] + w[1]*w[1] + w[2]*w[2] );
  double qw[4];
  if( wnorm != 0.0 ){
    double wdt05 = 0.5*wnorm*dt;
    double swdt = sin(wdt05)/wnorm;
    qw[0] = cos(wdt05);
    qw[1] = w[0]*swdt;
    qw[2] = w[1]*swdt;
    qw[3] = w[2]*swdt;
  }else{
    qw[0] = 1.0;
    qw[1] = 0.0;
    qw[2] = 0.0;
    qw[3] = 0.0;
  }
  
  double qp[4];
  qp[0] = this->q[0]*qw[0] - this->q[1]*qw[1] - this->q[2]*qw[2] - this->q[3]*qw[3];
  qp[1] = this->q[0]*qw[1]  +  qw[0]*this->q[1]  +  this->q[2]*qw[3] - this->q[3]*qw[2];
  qp[2] = this->q[0]*qw[2]  +  qw[0]*this->q[2]  +  this->q[3]*qw[1] - this->q[1]*qw[3];
  qp[3] = this->q[0]*qw[3]  +  qw[0]*this->q[3]  +  this->q[1]*qw[2] - this->q[2]*qw[1];
  
  // we compute the covariance matrix for the state prediction
  double dtdt2 = 0.5*dt*dt;
  double dtdtdt3 = dt*dt*dt/3.0;
  
  for(int j=0; j<3; j++){
    for(int i=0; i<3; i++) this->P[i+j*6] += this->Qw[i+j*3]*dtdtdt3;
    for(int i=3; i<6; i++) this->P[i+j*6] -= this->Qw[i-3+j*3]*dtdt2;
  }
  for(int j=3; j<6; j++){
    for(int i=0; i<3; i++) this->P[i+j*6] -= this->Qw[i+(j-3)*3]*dtdt2;
    for(int i=3; i<6; i++) this->P[i+j*6] += this->Qw[i-3+(j-3)*3]*dt;
  }
  
  double F[9];
  F[0] = -qw[2]*qw[2]-qw[3]*qw[3];    F[3] = qw[1]*qw[2]+qw[3]*qw[0];     F[6] = qw[1]*qw[3]-qw[2]*qw[0];
  F[1] = qw[1]*qw[2]-qw[3]*qw[0];     F[4] = -qw[1]*qw[1]-qw[3]*qw[3];    F[7] = qw[2]*qw[3]+qw[1]*qw[0];
  F[2] = qw[1]*qw[3]+qw[2]*qw[0];     F[5] = qw[2]*qw[3]-qw[1]*qw[0];     F[8] = -qw[1]*qw[1]-qw[2]*qw[2];
  
  F[0] += F[0] + 1.0;    F[3] += F[3];          F[6] += F[6];
  F[1] += F[1];          F[4] += F[4] + 1.0;    F[7] += F[7];
  F[2] += F[2];          F[5] += F[5];          F[8] += F[8] + 1.0;
  
  double M[36];
  M[0] = F[0];    M[6] = F[3];    M[12] = F[6];    M[18] = F[0]*dt;    M[24] = F[3]*dt;    M[30] = F[6]*dt;
  M[1] = F[1];    M[7] = F[4];    M[13] = F[7];    M[19] = F[1]*dt;    M[25] = F[4]*dt;    M[31] = F[7]*dt;
  M[2] = F[2];    M[8] = F[5];    M[14] = F[8];    M[20] = F[2]*dt;    M[26] = F[5]*dt;    M[32] = F[8]*dt;
  M[3] = 0.0;     M[9] = 0.0;     M[15] = 0.0;     M[21] = 1.0;        M[27] = 0.0;        M[33] = 0.0;
  M[4] = 0.0;     M[10] = 0.0;    M[16] = 0.0;     M[22] = 0.0;        M[28] = 1.0;        M[34] = 0.0;
  M[5] = 0.0;     M[11] = 0.0;    M[17] = 0.0;     M[23] = 0.0;        M[29] = 0.0;        M[35] = 1.0;
  
  double S[36];
  for(int i=0; i<6; i++){
    for(int j=0; j<6; j++){
      double sum = 0.0;
      for(int k=0; k<6; k++) sum += this->P[i+k*6]*M[j+k*6];
      S[i*6+j] = sum;
    }
  }
  for(int i=0; i<6; i++){
    for(int j=0; j<6; j++){
      double sum = 0.0;
      for(int k=0; k<6; k++) sum += M[i+k*6]*S[k*6+j];
      this->P[i+j*6] = sum;
    }
  }
  
  // we compute the measurement prediction
  double ap[3] = { qp[1]*qp[3]-qp[2]*qp[0] , qp[2]*qp[3]+qp[1]*qp[0] , -qp[1]*qp[1]-qp[2]*qp[2] };
  ap[0] += ap[0];
  ap[1] += ap[1];
  ap[2] += ap[2] + 1.0;
  
  F[0] = 0.0;       F[3] = -ap[2];    F[6] = ap[1];
  F[1] = ap[2];     F[4] = 0.0;       F[7] = -ap[0];
  F[2] = -ap[1];    F[5] = ap[0];     F[8] = 0.0;
  
  double H[36];
  H[0] = F[0];    H[6] = F[3];    H[12] = F[6];    H[18] = 0.0;    H[24] = 0.0;    H[30] = 0.0;
  H[1] = F[1];    H[7] = F[4];    H[13] = F[7];    H[19] = 0.0;    H[25] = 0.0;    H[31] = 0.0;
  H[2] = F[2];    H[8] = F[5];    H[14] = F[8];    H[20] = 0.0;    H[26] = 0.0;    H[32] = 0.0;
  H[3] = 0.0;     H[9] = 0.0;     H[15] = 0.0;     H[21] = 1.0;    H[27] = 0.0;    H[33] = 0.0;
  H[4] = 0.0;     H[10] = 0.0;    H[16] = 0.0;     H[22] = 0.0;    H[28] = 1.0;    H[34] = 0.0;
  H[5] = 0.0;     H[11] = 0.0;    H[17] = 0.0;     H[23] = 0.0;    H[29] = 0.0;    H[35] = 1.0;
  
  for(int i=0; i<6; i++){
    for(int j=0; j<6; j++){
      double sum = 0.0;
      for(int k=0; k<6; k++) sum += this->P[i+k*6]*H[j+k*6];
      M[i*6+j] = sum;
    }
  }
  
  for(int j=0; j<6; j++){
    for(int i=0; i<6; i++){
      double sum = 0.0;
      for(int k=0; k<6; k++) sum += H[i+k*6]*M[k*6+j];
      S[i+j*6] = sum;
    }
  }
  
  for(int j=0; j<3; j++){
    for(int i=0; i<3; i++) S[i+j*6] += this->Qa[i+j*3] + this->Ra[i+j*3];
  }
  for(int j=3; j<6; j++){
    for(int i=3; i<6; i++) S[i+j*6] += this->Rw[i-3+(j-3)*3];
  }
  
  // now we can compute the gain
  MEKF::solve( S , M );  // now K is stored in M
  
  // and update the state in the chart
  double dy[6] = { am[0]-ap[0] , am[1]-ap[1] , am[2]-ap[2] , wm[0]-this->w[0] , wm[1]-this->w[1] , wm[2]-this->w[2] };
  
  double dx[6];
  for(int i=0; i<6; i++){
    double sum = 0.0;
    for(int j=0; j<6; j++) sum += M[i*6+j]*dy[j];
    dx[i] = sum;
  }
  
  // the updated point in the chart is mapped to a quaternion
  this->fC2M( dx , qw );  // now delta is stored in qw
  
  this->q[0] = qp[0]*qw[0] - qp[1]*qw[1] - qp[2]*qw[2] - qp[3]*qw[3];
  this->q[1] = qp[0]*qw[1]  +  qw[0]*qp[1]  +  qp[2]*qw[3] - qp[3]*qw[2];
  this->q[2] = qp[0]*qw[2]  +  qw[0]*qp[2]  +  qp[3]*qw[1] - qp[1]*qw[3];
  this->q[3] = qp[0]*qw[3]  +  qw[0]*qp[3]  +  qp[1]*qw[2] - qp[2]*qw[1];
  
  // and the angular velocity is updated in the usual way
  this->w[0] += dx[3];
  this->w[1] += dx[4];
  this->w[2] += dx[5];
  
  // the covariance matrix is updated in the chart centered in qp
  for(int i=0; i<6; i++){
    for(int j=0; j<6; j++){
      double sum = 0.0;
      for(int k=0; k<6; k++) sum -= M[i*6+k]*H[k+j*6];
      S[i*6+j] = sum;
    }
  }
  for(int k=0; k<36; k+=7) S[k] += 1.0;
  
  for(int i=0; i<6; i++){
    for(int j=0; j<6; j++){
      double sum = 0.0;
      for(int k=0; k<6; k++) sum += S[i*6+k]*this->P[k+j*6];
      M[i+j*6] = sum;
    }
  }
  
#if defined CHART_UPDATE
  // finally we update the covariance matrix from the chart centered in qp
  // quaternion to the chart centered in the updated q quaternion
  this->chartUpdateMatrix( qw , H );  // now G is stored in H
  
  S[0] = H[0];    S[6] = H[3];    S[12] = H[6];    S[18] = 0.0;    S[24] = 0.0;    S[30] = 0.0;
  S[1] = H[1];    S[7] = H[4];    S[13] = H[7];    S[19] = 0.0;    S[25] = 0.0;    S[31] = 0.0;
  S[2] = H[2];    S[8] = H[5];    S[14] = H[8];    S[20] = 0.0;    S[26] = 0.0;    S[32] = 0.0;
  S[3] = 0.0;     S[9] = 0.0;     S[15] = 0.0;     S[21] = 1.0;    S[27] = 0.0;    S[33] = 0.0;
  S[4] = 0.0;     S[10] = 0.0;    S[16] = 0.0;     S[22] = 0.0;    S[28] = 1.0;    S[34] = 0.0;
  S[5] = 0.0;     S[11] = 0.0;    S[17] = 0.0;     S[23] = 0.0;    S[29] = 0.0;    S[35] = 1.0;
  
  for(int i=0; i<6; i++){
    for(int j=0; j<6; j++){
      double sum = 0.0;
      for(int k=0; k<6; k++) sum += M[i+k*6]*S[j+k*6];
      H[i*6+j] = sum;
    }
  }
  
  for(int j=0; j<6; j++){
    for(int i=0; i<6; i++){
      double sum = 0.0;
      for(int k=0; k<6; k++) sum += S[i+k*6]*H[k*6+j];
      M[i+j*6] = sum;
    }
  }
#endif
  
  // we avoid numerical instabilities
  double qnorm = sqrt( this->q[0]*this->q[0] + this->q[1]*this->q[1] + this->q[2]*this->q[2] + this->q[3]*this->q[3] );
  this->q[0] /= qnorm;
  this->q[1] /= qnorm;
  this->q[2] /= qnorm;
  this->q[3] /= qnorm;
  
  for(int i=0; i<6; i++){
    for(int j=0; j<6; j++) this->P[i+j*6] = 0.5*( M[i+j*6] + M[j+i*6] );
  }
  
  return;
}


// Method: Cholesky
// performs the Cholesky decomposition of a positive definite matrix ( S = L*L' )
// inputs:
//  S: 6x6 positive definite matrix to be decomposed (must be stored by columns)
// outputs:
//  S: the lower triangular matrix L (6x6) is overwritten in S (is stored by columns)
void MEKF::Cholesky( double* S ){
  // for each column
  for(int j=0; j<6; j++){
    double sum = 0.0;  //sum for the diagonal term
    // we first fill with 0.0 until diagonal
    for(int i=0; i<j; i++){
      S[i+j*6] = 0.0;
      //we can compute this sum at the same time
      sum += S[j+i*6]*S[j+i*6];
    }
    // now we compute the diagonal term
    S[j*7] = sqrt( S[j*7] - sum ); //S[j+j*m] = sqrt( S[j+j*m] - sum );
    // finally we compute the terms below the diagonal
    for(int i=j+1; i<6; i++){
      //first the sum
      double sum = 0.0;
      for(int k=0; k<j; k++){
        sum += S[i+k*6]*S[j+k*6];
      }
      //after the non-diagonal term
      S[i+j*6] = ( S[i+j*6] - sum )/S[j*7];
    }
  }//end j
  
  return;
}


// Method: solve
// solves the system of linear equations  K*S = M  for K
// inputs:
//  S: 6x6 positive definite matrix stored by columns
//  M: 6x6 matrix stored by rows
// outputs:
//  M: K (6x6) is stored by rows in the M memory space
void MEKF::solve( double* S , double* M ){
  // we first compute the Cholesky decomposition for transform the system from  K*S = M  into K*L*L' = M
  MEKF::Cholesky( S );
  
  double y[6];
  // then we take each pair of rows of K and M independently
  for(int i=0; i<6; i++){
    // first we solve (y*L' = M)
    for(int j=0; j<6; j++){
      double sum = 0.0;
      for(int k=0; k<j; k++){
        sum += y[k]*S[j+k*6];
      }
      y[j] = ( M[i*6+j] - sum )/S[j*7];
    }
    // now we solve (Ki*L = y)
    for(int j=5; j>-1; j--){
      double sum = 0.0;
      for(int k=j+1; k<6; k++){
        sum += M[i*6+k]*S[k+j*6];
      }
      M[i*6+j] = ( y[j] - sum )/S[j*7];
    }
  }
  
  return;
}




void MEKF::printMatrix( double* M , int n , int m ){
  for(int i=0; i<n; i++){
    for(int j=0; j<m; j++) std::cout << std::setw(19) << std::setfill(' ') << std::setprecision(10) << M[i+j*n];
    std::cout << "\n";
  }
  std::cout << "\n";
  
  return;
}



