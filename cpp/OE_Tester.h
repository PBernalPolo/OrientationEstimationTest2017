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
 * File:   OE_Tester.h
 * Author: P.Bernal-Polo
 * 
 * Created on February 19, 2017, 2:05 PM
 */


#ifndef OE_TESTER_H
#define OE_TESTER_H


// pi definition
#define PI 3.14159265358979323846


#include <math.h>  // round(), sqrt()
#include <stdlib.h>  // rand_r()
#include <iostream>  // input, output
#include <sstream>      // std::ostringstream
#include <fstream>  // input, output to files
#include <iomanip>  // io manipulation (set width, digits, ...)
#include <thread>  // to multithreading

#include "OrientationEstimator.h"


// OE_Tester: class used for testing the performance of an OrientationEstimator algorithm
//   
//   This class implements some methods with the objective of testing, in
//   an easy way, the performance of an algorithm of the
//   OrientationEstimator class
class OE_Tester {
public:
  // PUBLIC METHODS
  OE_Tester();
  OE_Tester(const OE_Tester& orig);
  virtual ~OE_Tester();
  
  // sets the title for files to save results
  void set_title( std::string titleIn );
  
  // sets the frequencies for which the performance of the filter will be evaluated.
  // the frequency vector introduced here will be modified choosing the
  // nearest frequencies that produce an update at obj.Tsim
  void set_f( double* fIn , int NfrequenciesIn );
  
  // sets the convergence threshold for the convergence step of the simulation
  void set_convergenceThreshold( double convergenceThresholdIn );
  
  // sets the maximum number of updates for the convergence step of the simulation
  void set_maxConvergenceUpdates( int maxConvergenceUpdatesIn );
  
  // sets the time for which the simulation is run
  void set_Tsim( double TsimIn );
  
  // sets the simulation steps carried before each OrientationEstimator update
  void set_simStepsPerUpdate( int dtdtsiminIn );
  
  // sets the number of trajectories generated for each frequency in order of testing the OrientationEstimator
  void set_Ntimes( int NtimesIn );
  
  // sets the variance for the process noise in the angular velocity
  void set_Qw( double QwIn );
  
  // sets the variance for the process noise in acceleration
  void set_Qa( double QaIn );
  
  // sets the variance for the angular velocity measurement
  void set_Rw( double RwIn );
  
  // sets the variance for the acceleration measurement
  void set_Ra( double RaIn );
  
  // uses the OE_Tester parameters to test an OrientationEstimator, and saves the results in a file
  void test( OrientationEstimator* myEstimator , double infoUpdateTime );
  
  // it perform the test() method in multiple threads (to improve the testing speed)
  // to compile this function we need to use the C++11 compiler:
  // properties->C++ Compiler->C++ Standard->C++11
  // and link a library:
  // properties->Linker->Libraries->Add Standard Library->Posix Threads
  void multithread_test( OrientationEstimator* myEstimator , double infoUpdateTime , int Nthreads );
  
  
private:
  // PRIVATE VARIABLES
  // title used to save results
  std::string title;
  // number of frequency samples
  int Nfrequencies;
  // vector of frequency samples
  double f[50];
  // threshold for which convergence is considered achieved (it is stored in radians, but introduced in degrees)
  double convergenceThreshold;
  // maximum number of updates when trying to achieve convergence
  int maxConvergenceUpdates;
  // simulation time (s)
  double Tsim;
  // (dt/dtsim) measure of how big is the update time step (dt) vs the simulation time step (dtsim)
  int dtdtsim;
  // times to repeat the simulation for each frequency
  int Ntimes;
  // angular velocity variance per unit of time (rad^2/s^3)
  double Qw;
  // acceleration variance (g^2)
  double Qa;
  // variance in measured angular velocity (rad^2/s^2)
  double Rw;
  // variance in measured acceleration (g^2)
  double Ra;
  
  
  // PRIVATE METHODS
  
  // used to save results of the test method
  void saveResults( int* Ncrashes , int* NnoConv , double* eT , double* eT2 , double* eG , double* eG2 , double* eQ , double* eQ2 );
  
  
  // PRIVATE STATIC METHODS
  
  // generates a trajectory step from a previous known state
  static void trajectoryStep( double* q , double* w , double dtsim , double nw , unsigned int* seed );
  
  // generates an IMU measurement from a known state
  static void IMU_Measurement( double* q , double* w , double na , double ra , double rw , unsigned int* seed , double* am , double* wm );
  
  // computes the angle between real and estimated gravity vectors
  static double thetaG( const double* qReal , const double* qEst );
  
  // computes the angle between real and estimated orientations
  static double thetaQ( const double* qr0 , const double* qr , const double* qe0 , const double* qe );
  
  // generates a sample of a normal distribution
  static double myNormalRandom( unsigned int* seed , double sigma );
  
  // checks if there has been a crash in the algorithm
  static bool hasCrashed( double* q );
  
  // a wrapper to use the test() method with multithreading
  static void testWrapper( OE_Tester* myOE_Tester , OrientationEstimator* myEstimator , double infoUpdateTime );
  
  
  static void printMatrix( double* M , int n , int m );
  
};

#endif /* OE_TESTER_H */

