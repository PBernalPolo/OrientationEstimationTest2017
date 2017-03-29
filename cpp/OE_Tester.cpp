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
 * File:   OE_Tester.cpp
 * Author: P.Bernal-Polo
 * 
 * Created on February 19, 2017, 2:05 PM
 */


#include "OE_Tester.h"


using namespace std;


OE_Tester::OE_Tester() {
  title = "simulationResult";
  // number of frequency samples
  Nfrequencies = 4;
  // vector of frequency samples
  f[0] = 1.0;    f[1] = 5.0;    f[2] = 10.0;    f[3] = 100.0;
  // threshold for which convergence is considered achieved (it is stored in radians, but introduced in degrees)
  convergenceThreshold = 3.0*PI/180.0;
  // maximum number of updates when trying to achieve convergence
  maxConvergenceUpdates = 1000;
  // simulation time (s)
  Tsim = 1.0;
  // (dt/dtsim) measure of how big is the update time step (dt) vs the simulation time step (dtsim)
  dtdtsim = 10;
  // times to repeat the simulation for each frequency
  Ntimes = 100;
  // angular velocity variance per unit of time (rad^2/s^3)
  Qw = 1.0e0;
  // acceleration variance (g^2)
  Qa = 1.0e-2;
  // variance in measured angular velocity (rad^2/s^2)
  Rw = 1.0e-4;
  // variance in measured acceleration (g^2)
  Ra = 1.0e-4;
}

OE_Tester::OE_Tester(const OE_Tester& orig) {
}

OE_Tester::~OE_Tester() {
}


// Method: set_title
// sets the title for files to save results
// inputs:
//  titleIn: title of the file to save results
// outputs:
void OE_Tester::set_title( std::string titleIn ){
  title = titleIn;
  
  return;
}


// Method: set_f
// sets the frequencies for which the performance of the filter will be evaluated.
// the frequency vector introduced here will be modified choosing the
// nearest frequencies that produce an update at obj.Tsim
// inputs:
//  fIn: vector of frequencies (Hz)
//  NfrequenciesIn: number of frequencies in fIn
void OE_Tester::set_f( double* fIn , int NfrequenciesIn ){
  // first the Nfrequencies
  if(  NfrequenciesIn < 0  ||  50 < NfrequenciesIn  ){
    std::cout << "Error in \"set_f()\": Nfrequencies must be between 1 and 50\n";
    throw 1;
  }
  Nfrequencies = NfrequenciesIn;
  // then the frequencies
  for(int k=0; k<Nfrequencies; k++){
    if( fIn[k] < 1.0/Tsim ){
      cout << " " << k << " " << fIn[k] << " " << Tsim << "\n";
      std::cout << "Error in \"set_f()\": frequencies must be greater than 1.0/Tsim\n";
      std::cout << "Is it possible that the variable Nfrequencies is not in agreement with the definition of f[]?";
      throw 1;
    }
  }
  for(int k=0; k<Nfrequencies; k++) f[k] = fIn[k];
  
  return;
}


// Method: set_convergenceThreshold
// sets the convergence threshold for the convergence step of the simulation
// inputs:
//  convergenceThresholdIn: degrees difference for which the convergence is considered achieved (degrees)
// outputs:
void OE_Tester::set_convergenceThreshold( double convergenceThresholdIn ){
  if( convergenceThresholdIn <= 0.0 ){
    std::cout << "Error in \"set_convergenceThreshold()\": convergenceThreshold must be positive\n";
    throw 1;
  }
  convergenceThreshold = convergenceThresholdIn*PI/180.0;
  
  return;
}


// Method: set_maxConvergenceUpdates
// sets the maximum number of updates for the convergence step of the simulation
// inputs:
//  maxConvergenceUpdatesIn: maximum number of updates for the convergence step
// outputs:
void OE_Tester::set_maxConvergenceUpdates( int maxConvergenceUpdatesIn ){
  if( maxConvergenceUpdatesIn < 1 ){
    std::cout << "Error in \"set_maxConvergenceUpdates()\": maxConvergenceUpdates can not be negative\n";
    throw 1;
  }
  maxConvergenceUpdates = maxConvergenceUpdatesIn;
  
  return;
}


// Method: set_Tsim
// sets the time for which the simulation is run
// inputs:
//  TsimIn: period of time for which the simulation is run (s)
// outputs:
void OE_Tester::set_Tsim( double TsimIn ){
  double minf = 1.0e9;
  for(int k=0; k<Nfrequencies; k++){
    if( f[k] < minf ) minf = f[k];
  }
  if( TsimIn < 1.0/minf ){
    std::cout << "Error in \"set_Tsim()\": Tsim must be greater than 1.0/fmin\n";
    throw 1;
  }
  Tsim = TsimIn;
  
  return;
}


// Method: set_simStepsPerUpdate
// sets the simulation steps carried before each OrientationEstimator update
// inputs:
//  dtdtsiminIn: number of simulation steps carried before each update (that turns out to be equal to dt/dtsim)
// outputs:
void OE_Tester::set_simStepsPerUpdate( int dtdtsimIn ){
  if( dtdtsimIn < 1 ){
    std::cout << "Error in \"set_simStepsPerUpdate()\": dtdtsim must be positive\n";
    throw 1;
  }
  dtdtsim = dtdtsimIn;
  
  return;
}


// Method: set_Ntimes
// sets the number of trajectories generated for each frequency in order of testing the OrientationEstimator
// inputs:
//  NtimesIn: number of trajectories generated for each frequency
// outputs:
void OE_Tester::set_Ntimes( int NtimesIn ){
  if( NtimesIn < 1 ){
    std::cout << "Error in \"set_Ntimes()\": Ntimes must be positive\n";
    throw 1;
  }
  Ntimes = NtimesIn;
  
  return;
}


// Method: set_Qw
// sets the variance for the process noise in the angular velocity
// inputs:
//  QwIn: variance for the process noise in the angular velocity per unit of time (rad^2/s^3)
// outputs:
void OE_Tester::set_Qw( double QwIn ){
  if( QwIn <= 0.0 ){
    std::cout << "Error in \"set_Qw()\": Qw must be positive\n";
    throw 1;
  }
  Qw = QwIn;
  
  return;
}


// Method: set_Qa
// sets the variance for the process noise in acceleration
// inputs:
//  QaIn: variance for the process noise in the acceleration (g^2)
// outputs:
void OE_Tester::set_Qa( double QaIn ){
  if( QaIn <= 0.0 ){
    std::cout << "Error in \"set_Qa()\": Qa must be positive\n";
    throw 1;
  }
  Qa = QaIn;
  
  return;
}


// Method: set_Rw
// sets the variance for the angular velocity measurement
// inputs:
//  RwIn: variance of the angular velocity measurement (rad^2/s^2)
// outputs:
void OE_Tester::set_Rw( double RwIn ){
  if( RwIn <= 0.0 ){
    std::cout << "Error in \"set_Rw()\": Rw must be positive\n";
    throw 1;
  }
  Rw = RwIn;
  
  return;
}


// Method: set_Ra
// sets the variance for the acceleration measurement
// inputs:
//  RaIn: variance for the acceleration measurement (g^2)
// outputs:
void OE_Tester::set_Ra( double RaIn ){
  if( RaIn <= 0.0){
    std::cout << "Error in \"set_Ra()\": Ra must be positive\n";
    throw 1;
  }
  Ra = RaIn;
  
  return;
}


// Method: test
// uses the OE_Tester parameters to test an OrientationEstimator, and
// saves the results in a file
// inputs:
//  myEstimator: the OrientationEstimator object for which we want to find its performance
//  infoUpdateTime: time step to show information about the simulation progress. If it is negative, no information will be shown
// outputs:
void OE_Tester::test( OrientationEstimator* myEstimator , double infoUpdateTime ){
  // first of all, let me write about the decisions taken for this simulation.
  // The first thought was to generate a sequence of states that generate a smooth
  // and continuous trajectory for the orientations.
  // Then we would take simulated measurements, and used these sequences for updating
  // our estimator. There are two major problems with this approach:
  // - we can not assert the coincidence of the final step of the simulation with the final update
  // - we can not assert the coincidence of all the estimation updates for all the
  // frequencies, with a given time step of the sequence of simulated states
  // A second approach to overcome these problems is to generate a different trajectory
  // for each frequency. It is done defining an entire quantity dtdtsim, that determines
  // the simulation steps performed before each estimator update (asserting the equality
  // "n*dtsim = dt", with n an integer).
  // Then, the frequencies are transformed to satisfy the equality
  // "m*dt = Tsim", with m an integer.
  // After explaining this, we can start with the code
  
  // we require the condition that the final update happens at the end of the simulation:
  //    n*dt = Tsim  =>  f = 1.0/dt = n/Tsim = round(Tsim/dt)/Tsim = round(Tsim*f)/Tsim
  for(int nf=0; nf<Nfrequencies; nf++){
    f[nf] = round( Tsim*f[nf] )/Tsim;
  }
  // we do not want repeated frequencies
  for(int nf1=0; nf1<Nfrequencies; nf1++){
    for(int nf2=nf1+1; nf2<Nfrequencies; nf2++){
      if( round(Tsim*f[nf1]) == round(Tsim*f[nf2]) ){
        Nfrequencies--;
        for(int nf=nf2; nf<Nfrequencies; nf++) f[nf] = f[nf+1];
        nf2--;
      }
    }
  }
  
  // we compute the sigmas of the measurement covariances
  double rw = sqrt( Rw );
  double ra = sqrt( Ra );
  
  // we will save our error definitions in these vectors (general definitions: outside threads)
  int Ncrashes[Nfrequencies];
  int NnoConvergence[Nfrequencies];
  double TConv[Nfrequencies];
  double TConv2[Nfrequencies];
  double errorG[Nfrequencies];
  double errorG2[Nfrequencies];
  double errorQ[Nfrequencies];
  double errorQ2[Nfrequencies];
  for(int nf=0; nf<Nfrequencies; nf++){
    Ncrashes[nf] = 0;
    NnoConvergence[nf] = 0;
    TConv[nf] = 0.0;
    TConv2[nf] = 0.0;
    errorG[nf] = 0.0;
    errorG2[nf] = 0.0;
    errorQ[nf] = 0.0;
    errorQ2[nf] = 0.0;
  }
  
  // we will use this variable to display some information
  std::string msg = "";
  
  // we initialize the seed
  unsigned int seed = std::hash<std::thread::id>()(std::this_thread::get_id()); //hasher( this_thread::get_id() ); //time(NULL);
  
  // we take the initial time to give time estimations
  int beginingTime = time(NULL);
  int lastTime = beginingTime;
  
  // we do it several times to get a good performance estimation
  for(int n=0; n<Ntimes; n++){
    // we test for each frequency
    for(int nf=0; nf<Nfrequencies; nf++){
      // we compute the update time step
      double dt = 1.0/f[nf];
      
      // we generate random sigmas for the process
      double nw = myNormalRandom( &seed , sqrt(Qw) );
      double na = myNormalRandom( &seed , sqrt(Qa) );
      
      // we take a random initial orientation, and zero angular velocity
      double q0[4];
      for(int i=0; i<4; i++) q0[i] = myNormalRandom( &seed , 1.0 );
      double q0norm = sqrt( q0[0]*q0[0] + q0[1]*q0[1] + q0[2]*q0[2] + q0[3]*q0[3] );
      for(int k=0; k<4; k++) q0[k] /= q0norm;
      double w0[3] = { 0.0 , 0.0 , 0.0 };
      
      // and we initialize the estimator
      double RwIn[9], RaIn[9];
      for(int k=0; k<9; k++){
        RwIn[k] = 0.0;
        RaIn[k] = 0.0;
      }
      for(int k=0; k<9; k+=4){
        RwIn[k] = Rw;
        RaIn[k] = Ra;
      }
      myEstimator->initialize( RwIn , RaIn );
      
      // the first simulation part is to maintain the IMU static until it reaches
      // convergence, too many updates, or a crash
      double theta = 1.0e2;
      int Nupdates = 0;
      bool crashed = false;
      while(  theta > convergenceThreshold  &&  Nupdates < maxConvergenceUpdates  ){
        // we compute the simulated measurement
        double am[3] , wm[3];
        IMU_Measurement( q0 , w0 , na , ra , rw , &seed , am , wm );
        // and we update the filter with the measurements
        myEstimator->updateIMU( am , wm , dt );
        Nupdates += 1;
        // we take the estimated orientation, and we check if the algorithm has crashed
        double qEst[4];
        myEstimator->get_q( qEst );
        if( hasCrashed(qEst) ){
          crashed = true;
          break;
        }
        // we compute the error in the gravity estimation
        theta = thetaG( q0 , qEst );
      }
      // we check if there has been a crash
      if( crashed ){
        Ncrashes[nf] += 1;
        continue;
      }
      // we check if there has been convergence
      if( Nupdates >= maxConvergenceUpdates ){
        // if not, we add one to the no-convergences, and we go with the next frequency
        NnoConvergence[nf] += 1;
        continue;
      }
      
      // we go ahead with the metrics computations, and with the simulation
      TConv[nf] += Nupdates;
      TConv2[nf] += Nupdates*Nupdates;
      
      // we save the initial estimated quaternion
      double qe0[4];
      myEstimator->get_q( qe0 );
      
      // we take the initial state
      double q[4] = { q0[0] , q0[1] , q0[2] , q0[3] };
      double w[3] = { w0[0] , w0[1] , w0[2] };
      
      // and the second step of the simulation is to generate a random
      // orientation trajectory, and try to estimate it
      //   we need the simulation time step
      double dtsim = dt/dtdtsim;
      //   and the number of updates
      Nupdates = round( Tsim*f[nf] );
      //   we also need the integral of thetaG, thetaG^2, thetaQ, and thetaQ^2
      double thG = 0.0 ,  thG2 = 0.0 ,  thQ = 0.0 ,  thQ2 = 0.0;
      //   now we can start iterating
      for(int nup=0; nup<Nupdates; nup++){
        // first we iterate in the simulation (trajectory generation)
        for(int nt=0; nt<dtdtsim; nt++){
          trajectoryStep( q , w , dtsim , nw , &seed );
        }
        // then we simulate the measurement
        double am[3], wm[3];
        IMU_Measurement( q , w , na , ra , rw , &seed , am , wm );
        // finally we update the estimator with the measurement
        myEstimator->updateIMU( am , wm , dt );
        // and we add the errors
        //   we get the quaternion
        double qEst[4];
        myEstimator->get_q( qEst );
        if( hasCrashed(qEst) ){
          crashed = true;
          break;
        }
        //   angle between real and estimated gravity vector
        double theta = thetaG( q , qEst );
        thG += theta;
        thG2 += theta*theta;
        //   angle between orientations (defined as the angle we have to rotate)
        theta = thetaQ( q0 , q , qe0 , qEst );
        thQ += theta;
        thQ2 += theta*theta;
        // we repeat this for Nupdates
      }
      // we check if there has been a crash
      if( crashed ){
        Ncrashes[nf] += 1;
        continue;
      }
      
      // now we can compute the error definitions
      double dtTsim = dt/Tsim;
      //   error in gravity estimation
      errorG[nf] += thG*dtTsim;
      errorG2[nf] += thG2*dtTsim;
      //   error in orientation
      errorQ[nf] += thQ*dtTsim;
      errorQ2[nf] += thQ2*dtTsim;
      
    }
    
    // if enough time has passed
    if( infoUpdateTime > 0.0 ){
      int currentTime = time(NULL);
      if(  currentTime - lastTime  >  infoUpdateTime  ){
        // we display some information about resting time, and about the simulation progress
        double eTime2end = (currentTime-beginingTime)*((double)(Ntimes-n))/n;
        int eTime2endD = floor( eTime2end/86400.0 );
        int eTime2endH = floor( ( eTime2end-eTime2endD*86400.0 )/3600.0 );
        int eTime2endM = floor( ( eTime2end-eTime2endD*86400.0-eTime2endH*3600.0 )/60.0 );
        int eTime2endS = ceil( eTime2end-eTime2endD*86400.0-eTime2endH*3600.0-eTime2endM*60.0 );
        double proportion = ((double)n)/Ntimes;
        
        std::ostringstream message;
        message << "[";
        for(int i=0; i<round(proportion*100.0); i++) message << "-";
        for(int i=0; i<100-round(proportion*100.0); i++) message << " ";
        message << "] (" << proportion << ") Estimated remaining time: ";
        if( eTime2endD > 0 ) message << eTime2endD << " d ";
        if( eTime2endH > 0 ) message << eTime2endH << " h ";
        if( eTime2endM > 0 ) message << eTime2endM << " m ";
        message << eTime2endS << " s                      ";
        
        int msgLength = msg.length();
        msg = message.str();
        for(int i=0; i<msgLength; i++) std::cout << "\b";
        std::cout << msg << std::flush;
        
        // we update lastTime for the next infoUpdate
        lastTime = currentTime;
      }
    }
    
  }
  
  // we save the results, but first we correct the measurements
  for(int nf=0; nf<Nfrequencies; nf++){
    TConv[nf] /= f[nf];
    TConv2[nf] /= f[nf]*f[nf];
    errorG[nf] *= 180.0/PI;
    errorG2[nf] *= 180.0/PI*180.0/PI;
    errorQ[nf] *= 180.0/PI;
    errorQ2[nf] *= 180.0/PI*180.0/PI;
  }
  saveResults( Ncrashes , NnoConvergence , TConv , TConv2 , errorG , errorG2 , errorQ , errorQ2 );
  
  // if the user want (and to not printing on all threads)
  if( infoUpdateTime > 0.0 ){
    // we print the time taken
    int currentTime = time(NULL);
    double eTime2end = currentTime-beginingTime;
    int eTime2endD = floor( eTime2end/86400.0 );
    int eTime2endH = floor( ( eTime2end-eTime2endD*86400.0 )/3600.0 );
    int eTime2endM = floor( ( eTime2end-eTime2endD*86400.0-eTime2endH*3600.0 )/60.0 );
    int eTime2endS = ceil( eTime2end-eTime2endD*86400.0-eTime2endH*3600.0-eTime2endM*60.0 );
    
    std::ostringstream message;
    int msgLength = msg.length();
    for(int i=0; i<msgLength; i++) message << "\b";
    message << "Completed. Time taken: ";
    if( eTime2endD > 0 ) message << eTime2endD << " d ";
    if( eTime2endH > 0 ) message << eTime2endH << " h ";
    if( eTime2endM > 0 ) message << eTime2endM << " m ";
    message << eTime2endS << " s                      ";
    for(int i=0; i<msgLength; i++) message << " ";
    message << "\n";
    
    std::cout << message.str() << std::flush;
    
  }
  
}


// Method: saveResults
// used to save results of the test method
// inputs:
//  Ncrashes: vector (for each frequency) with number of algorithm crashes
//  NnoConv: vector (for each frequency) with number of non-achieved convergences at the second step of the simulation
//  eTs: vector (for each frequency) with the sum of convergence times
//  eT2s: vector (for each frequency) with the sum of squared convergence times
//  eGs: vector (for each frequency) with the sum of errors in gravity estimations
//  eG2s: vector (for each frequency) with the sum of squared errors in gravity estimations
//  eQs: vector (for each frequency) with the sum of errors in orientation estimations
//  eQ2s: vector (for each frequency) with the sum of squared errors in orientation estimations
// outputs:
void OE_Tester::saveResults( int* Ncrashes , int* NnoConvs , double* eTs , double* eT2s , double* eGs , double* eG2s , double* eQs , double* eQ2s ){
  // we open the file
  std::ofstream file;
  std::string myString = this->title + ".dat";
  file.open( myString.c_str() );
  
  // we set up the format
  file.precision(10);
  
  // we save some information about the tester
  file << "# title = " << this->title << "\n";
  file << "# convergenceThreshold = " << this->convergenceThreshold*180.0/PI << "\n";
  file << "# maxConvergenceUpdates = " << this->maxConvergenceUpdates << "\n";
  file << "# Tsim = " << this->Tsim << "\n";
  file << "# dtdtsim = " << this->dtdtsim << "\n";
  file << "# Ntimes = " << this->Ntimes << "\n";
  file << "# Qw = " << this->Qw << "\n";
  file << "# Qa = " << this->Qa << "\n";
  file << "# Rw = " << this->Rw << "\n";
  file << "# Ra = " << this->Ra << "\n";
  
  // now, for each frequency we save:
  file << "# frequency (Hz) |";
  file << " n crashes | n no convergences |";
  file << " mean convergence time (s) | sigma convergence time (s) | sigma mean convergence time (s) |";
  file << " mean error in gravity (degrees) | sigma of error in gravity (degrees) | sigma mean error in gravity (degrees) |";
  file << " mean error in orientation (degrees) | sigma of error in orientation (degrees) | sigma mean error in orientation (degrees) \n";
  for(int nf=0; nf<this->Nfrequencies; nf++){
    // first we need the number of samples
    int Nsamples = this->Ntimes - NnoConvs[nf];
    if( Nsamples == 0 ) Nsamples = 1;
    // - frequency (Hz)
    file << f[nf];
    // - number of crashes
    file << " " << Ncrashes[nf];
    // - number of no convergences
    file << " " << NnoConvs[nf];
    // - mean of the convergence time (s)
    double meT = eTs[nf]/Nsamples;
    file << " " << meT;
    // - sigma of the convergence time (s)
    double seT = sqrt( max( eT2s[nf]/Nsamples - meT*meT , 0.0 ) );
    file << " " << seT;
    // - sigma of the computation of the mean convergence time (s)
    double smeT = seT/sqrt(Nsamples);
    file << " " << smeT;
    // - mean of the error in gravity estimation (degrees)
    double meG = eGs[nf]/Nsamples;
    file << " " << meG;
    // - sigma of the error in gravity estimation (degrees)
    double seG = sqrt( max( eG2s[nf]/Nsamples - meG*meG , 0.0 ) );
    file << " " << seG;
    // - sigma of the mean error in gravity estimation (degrees)
    double smeG = seG/sqrt(Nsamples);
    file << " " << smeG;
    // - mean of the error in orientation estimation (degrees)
    double meQ = eQs[nf]/Nsamples;
    file << " " << meQ;
    // - sigma of the computation of the mean error in orientation estimation (degrees)
    double seQ = sqrt( max( eQ2s[nf]/Nsamples - meQ*meQ , 0.0 ) );
    file << " " << seQ;
    // - sigma of the computation of the mean error in orientation estimation (degrees)
    double smeQ = seQ/sqrt(Nsamples);
    file << " " << smeQ;
    file << "\n";
  }
  file << "\n";
  
  file.close();
  
  return;
}


// Method: testWrapper
// a wrapper to use the test() method with multithreading
// inputs:
//  myOE_Tester: pointer to the OE_Tester object with the testing parameters
//  myEstimator: pointer to the OrientationEstimator object that we want to test
//  infoUpdateTime: the infoUpdateTime variable to use in the test() method
// outputs:
void OE_Tester::testWrapper( OE_Tester* myOE_Tester , OrientationEstimator* myEstimator , double infoUpdateTime ){
  
  myOE_Tester->test( myEstimator , infoUpdateTime );
  
  return;
}


// Method: multithread_test
// it perform the test() method in multiple threads (to improve the testing speed)
// to compile this function we need to use the C++11 compiler:
// properties->C++ Compiler->C++ Standard->C++11
// and link a library:
// properties->Linker->Libraries->Add Standard Library->Posix Threads
// inputs:
//  myEstimator: pointer to the OrientationEstimator object to test
//  infoUpdateTime: time step to show information about the simulation progress. If it is negative, no information will be shown (only information about one thread will be shown)
//  Nthreads: number of threads to perform the test
// outputs:
void OE_Tester::multithread_test( OrientationEstimator* myEstimator , double infoUpdateTime , const int Nthreads ){
  
  if(Nthreads > 0){
    // we create arrays of pointers to OrientationEstimators, and OE_Testers
    OrientationEstimator *estimator[Nthreads];
    OE_Tester *tester[Nthreads];
    
    // we create deep copies of the objects
    for(int i=0; i<Nthreads; i++){
      estimator[i] = myEstimator->copy();
      tester[i] = new OE_Tester( *this );
      tester[i]->Nfrequencies = this->Nfrequencies;
      for(int nf=0; nf<Nfrequencies; nf++) tester[i]->f[nf] = this->f[nf];
      tester[i]->convergenceThreshold = this->convergenceThreshold;
      tester[i]->maxConvergenceUpdates = this->maxConvergenceUpdates;
      tester[i]->Tsim = this->Tsim;
      tester[i]->dtdtsim = this->dtdtsim;
      tester[i]->Qw = this->Qw;
      tester[i]->Qa = this->Qa;
      tester[i]->Rw = this->Rw;
      tester[i]->Ra = this->Ra;
    }
    // but they will have different title and Ntimes
    int threadTimes = floor( ((double)Ntimes)/Nthreads );
    int totalTimes = 0;
    for(int i=1; i<Nthreads; i++){
      std::ostringstream theTitle;
      theTitle << this->title << i;
      tester[i]->title = theTitle.str();
      tester[i]->Ntimes = threadTimes;
      totalTimes += threadTimes;
    }
    tester[0]->title = this->title + "0";
    tester[0]->Ntimes = this->Ntimes-totalTimes;
    
    // now we create the threads
    std::thread *myThread[Nthreads];
    for(int i=1; i<Nthreads; i++) myThread[i] = new std::thread( OE_Tester::testWrapper , tester[i] , estimator[i] , -1.0 );
    myThread[0] = new std::thread( OE_Tester::testWrapper , tester[0] , estimator[0] , infoUpdateTime );
    
    // and synchronize threads
    for(int i=1; i<Nthreads; i++) myThread[i]->join();
    myThread[0]->join();
    
    // now we can release the memory of the estimators and the threads
    for(int i=0; i<Nthreads; i++){
      delete estimator[i];
      delete myThread[i];
    }
    
    // after completing all the threads, we combine the information
    //   we are interested in the next values
    int Ncrashes[Nfrequencies];
    int NnoConv[Nfrequencies];
    double eTs[Nfrequencies];
    double eT2s[Nfrequencies];
    double eGs[Nfrequencies];
    double eG2s[Nfrequencies];
    double eQs[Nfrequencies];
    double eQ2s[Nfrequencies];
    for(int nf=0; nf<Nfrequencies; nf++){
      Ncrashes[nf] = 0;
      NnoConv[nf] = 0;
      eTs[nf] = 0.0;
      eT2s[nf] = 0.0;
      eGs[nf] = 0.0;
      eG2s[nf] = 0.0;
      eQs[nf] = 0.0;
      eQ2s[nf] = 0.0;
    }
    //   for each file created by each thread, we read the corresponding values
    for(int i=0; i<Nthreads; i++){
      // we open the file
      std::string theTitle = tester[i]->title + ".dat";
      ifstream theFile( theTitle.c_str() );
      // we skip lines starting by '#'
      string theLine = "#";
      while( theLine[0] == '#' ) getline( theFile , theLine );
      // and we read the rest
      for(int nf=0; nf<Nfrequencies; nf++){
        stringstream theStreamLine = stringstream(theLine);
        int nc, nnc;
        double dat, mean, sigma;
        // first we skip the frequency, and get the number of crashes and no-convergences
        theStreamLine >> dat;
        theStreamLine >> nc;
        Ncrashes[nf] += nc;
        theStreamLine >> nnc;
        NnoConv[nf] += nnc;
        // then, the number of samples is
        int Nsamples = tester[i]->Ntimes - nc - nnc;
        if( Nsamples == 0 ) Nsamples = 1;
        // convergence time
        theStreamLine >> mean;
        eTs[nf] += mean*Nsamples;
        theStreamLine >> sigma;
        eT2s[nf] += ( sigma*sigma + mean*mean )*Nsamples;
        theStreamLine >> dat;  // we skip the sigma in the mean
        // gravity error
        theStreamLine >> mean;
        eGs[nf] += mean*Nsamples;
        theStreamLine >> sigma;
        eG2s[nf] += ( sigma*sigma + mean*mean )*Nsamples;
        theStreamLine >> dat;  // we skip the sigma in the mean
        // orientation error
        theStreamLine >> mean;
        eQs[nf] += mean*tester[i]->Ntimes;
        theStreamLine >> sigma;
        eQ2s[nf] += ( sigma*sigma + mean*mean )*Nsamples;
        theStreamLine >> dat;  // we skip the sigma in the mean
        // and we get the new line
        getline( theFile , theLine );
      }
      // we close the file
      theFile.close();
      
      // and we delete it
      std::remove( theTitle.c_str() );
    }
    
    //   finally we store the combined values
    saveResults( Ncrashes , NnoConv , eTs , eT2s , eGs , eG2s , eQs , eQ2s );
    
    // and we release the memory of the testers
    for(int i=0; i<Nthreads; i++) delete tester[i];
    
  }else{
    test( myEstimator , infoUpdateTime );
  }
  
  return;
}


// Method: trajectoryStep
// generates a trajectory step from a previous known state
// inputs:
//  q: quaternion describing the real orientation
//  w: angular velocity measured in the sensor reference frame (rad/s)
//  dtsim: time step of the simulation (s)
//  nw: standard deviation of the process noise for the angular velocity (rad*s^(-3/2))
// outputs:
//  q: new quaternion describing the real orientation of the system (it satisfies qnew = q*qw)
//  w: new angular velocity (rad/s)
void OE_Tester::trajectoryStep( double* q , double* w , double dtsim , double nw , unsigned int* seed ){
  // first of all we generate three random numbers normally distributed
  double sqrtdtsim = sqrt(dtsim);
  for(int i=0; i<3; i++) w[i] += myNormalRandom( seed , nw )*sqrtdtsim;
  
  // we compute the next orientation using the previous orientation, and the angular velocity
  //   w norm computation
  double wnorm = sqrt( w[0]*w[0] + w[1]*w[1] + w[2]*w[2] );
  //   we compute qw
  double qw[4];
  if( wnorm != 0.0 ){
    double wdt05 = 0.5*wnorm*dtsim;
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
  //   we compute the new state (q*qw,w)
  double qn[4];
  qn[0] = q[0]*qw[0] - q[1]*qw[1] - q[2]*qw[2] - q[3]*qw[3];
  qn[1] = q[0]*qw[1]  +  qw[0]*q[1]  +  q[2]*qw[3] - q[3]*qw[2];
  qn[2] = q[0]*qw[2]  +  qw[0]*q[2]  +  q[3]*qw[1] - q[1]*qw[3];
  qn[3] = q[0]*qw[3]  +  qw[0]*q[3]  +  q[1]*qw[2] - q[2]*qw[1];
  // we make sure that the quaternion is normalized
  double qnorm = sqrt( qn[0]*qn[0] + qn[1]*qn[1] + qn[2]*qn[2] + qn[3]*qn[3] );
  q[0] = qn[0]/qnorm;
  q[1] = qn[1]/qnorm;
  q[2] = qn[2]/qnorm;
  q[3] = qn[3]/qnorm;
  
  return;
}


// Method: IMU_Measurement
// generates an IMU measurement from a known state
// inputs:
//  q: quaternion describing the orientation of the system (transform vectors from the sensor reference frame to the external reference frame)
//  w: real angular velocity measured in the sensor reference frame (rad/s)
//  na: standard deviation of the process noise for the acceleration (g)
//  ra: standard deviation of the accelerometer measurements (g)
//  rw: standard deviation of the gyroscope measurements (rad/s)
// outputs:
//  am: simulated accelerometer measurement (g)
//  wm: simulated gyroscope measurement (rad/s)
void OE_Tester::IMU_Measurement( double* q , double* w , double na , double ra , double rw , unsigned int* seed , double* am , double* wm ){
  // we compute the transposed rotation matrix
  double RT[9];
  RT[0] = -q[2]*q[2]-q[3]*q[3];    RT[3] = q[1]*q[2]+q[3]*q[0];     RT[6] = q[1]*q[3]-q[2]*q[0];
  RT[1] = q[1]*q[2]-q[3]*q[0];     RT[4] = -q[1]*q[1]-q[3]*q[3];    RT[7] = q[2]*q[3]+q[1]*q[0];
  RT[2] = q[1]*q[3]+q[2]*q[0];     RT[5] = q[2]*q[3]-q[1]*q[0];     RT[8] = -q[1]*q[1]-q[2]*q[2];
  
  RT[0] += RT[0] + 1.0;    RT[3] += RT[3];          RT[6] += RT[6];
  RT[1] += RT[1];          RT[4] += RT[4] + 1.0;    RT[7] += RT[7];
  RT[2] += RT[2];          RT[5] += RT[5];          RT[8] += RT[8] + 1.0;
  
  // then we generate three random numbers normally distributed
  double a[3], nra[3], nrw[3];
  for(int i=0; i<3; i++){
    a[i] = myNormalRandom( seed , na );
    nra[i] = myNormalRandom( seed , ra );
    nrw[i] = myNormalRandom( seed , rw );
  }
  
  // we obtain the simulated measurements
  //   am = RT*( a + [0; 0; 1] ) + normrnd( 0 , ra , [3 1] );
  a[2] += 1.0;
  for(int i=0; i<3; i++){
    double sum = nra[i];
    for(int j=0; j<3; j++) sum += RT[i+j*3]*a[j];
    am[i] = sum;
  }
  //   wm = w + normrnd( 0 , rw , [3 1] );
  for(int i=0; i<3; i++) wm[i] = w[i] + nrw[i];
  
  return;
}


// Method: thetaG
// computes the angle between real and estimated gravity vectors
// inputs:
//  qReal: real quaternion describing the orientation of the system
//  qEst: estimated quaternion describing the orientation of the system
//  (these quaternions transform vectors from the sensor reference frame to the external reference frame)
// outputs:
//  theta: angle between real and estimated gravity vectors (rad)
double OE_Tester::thetaG( const double* qReal , const double* qEst ){
  // real gravity vector
  double gReal[3];
  gReal[0] = qReal[1]*qReal[3]-qReal[2]*qReal[0];
  gReal[1] = qReal[2]*qReal[3]+qReal[1]*qReal[0];
  gReal[2] = 0.5-qReal[1]*qReal[1]-qReal[2]*qReal[2];
  gReal[0] += gReal[0];
  gReal[1] += gReal[1];
  gReal[2] += gReal[2];
  // estimated gravity vector
  double gEst[3];
  gEst[0] = qEst[1]*qEst[3]-qEst[2]*qEst[0];
  gEst[1] = qEst[2]*qEst[3]+qEst[1]*qEst[0];
  gEst[2] = 0.5-qEst[1]*qEst[1]-qEst[2]*qEst[2];
  gEst[0] += gEst[0];
  gEst[1] += gEst[1];
  gEst[2] += gEst[2];
  double gRgE = gReal[0]*gEst[0] + gReal[1]*gEst[1] + gReal[2]*gEst[2];
  if( gRgE > 1.0 ) gRgE = 1.0;
  if( gRgE < -1.0 ) gRgE = -1.0;
  
  // angle between real and estimated gravity vectors (rads)
  return acos( gRgE );
}


// Function: thetaQ
// computes the angle between real and estimated orientations
// inputs:
//  qr0: initial real quaternion describing the orientation in the simulation
//  qr: current real quaternion describing the orientation in the simulation
//  qe0: initial estimated quaternion describing the orientation in the simulation
//  qe: current estimated quaternion describing the orientation in the simulation
// (these quaternions transform vectors from the sensor reference frame to the external reference frame)
// outputs:
//  theta: angle between real and estimated orientations (rad)
double OE_Tester::thetaQ( const double* qr0 , const double* qr , const double* qe0 , const double* qe ){
  // compute the orientation difference between the initial and final real orientations
  double deltar[4];
  deltar[0] = qr0[0]*qr[0] + qr0[1]*qr[1] + qr0[2]*qr[2] + qr0[3]*qr[3];
  deltar[1] = qr0[0]*qr[1] - qr[0]*qr0[1] - qr0[2]*qr[3] + qr0[3]*qr[2];
  deltar[2] = qr0[0]*qr[2] - qr[0]*qr0[2] - qr0[3]*qr[1] + qr0[1]*qr[3];
  deltar[3] = qr0[0]*qr[3] - qr[0]*qr0[3] - qr0[1]*qr[2] + qr0[2]*qr[1];
  // compute the orientation difference between the initial and final estimated orientations
  double deltae[4];
  deltae[0] = qe0[0]*qe[0] + qe0[1]*qe[1] + qe0[2]*qe[2] + qe0[3]*qe[3];
  deltae[1] = qe0[0]*qe[1] - qe[0]*qe0[1] - qe0[2]*qe[3] + qe0[3]*qe[2];
  deltae[2] = qe0[0]*qe[2] - qe[0]*qe0[2] - qe0[3]*qe[1] + qe0[1]*qe[3];
  deltae[3] = qe0[0]*qe[3] - qe[0]*qe0[3] - qe0[1]*qe[2] + qe0[2]*qe[1];
  // compute the angle between differences of real and estimated orientations (rad)
  double delta0re = deltar[0]*deltae[0] + deltar[1]*deltae[1] + deltar[2]*deltae[2] + deltar[3]*deltae[3];
  //   the next computation only makes sense for a positive, and less than 1, delta0re
  if( delta0re < 0.0 ) delta0re = -delta0re;
  if( delta0re > 1.0 ) delta0re = 1.0;
  
  return 2.0*acos( delta0re );
}


// Method: myNormalRandom
// generates a sample of a normal distribution
// inputs:
//  seed: the seed for the rand_r() method that produces uniform distributed samples in the interval [0,1]
//  sigma: standard deviation of the normal distribution
// outputs:
//  rn: sample of the normal distribution (double)
double OE_Tester::myNormalRandom( unsigned int* seed , double sigma ){
  // we generate two uniform random doubles
  double urand1 = (rand_r( seed )%1000000 + 1)/1000000.0;
  double urand2 = (rand_r( seed )%1000000 + 1)/1000000.0;
  // now we build the normal random double from the other two, and we return it
  return sigma*sqrt(-2.0*log(urand1))*cos(2.0*PI*urand2);
}


// Function: hasCrashed
// checks if there has been a crash in the algorithm
// inputs:
//  q: quaternion computed with the algorithm
// outputs:
//  crashed: boolean variable. It will be true if it has crashed; false if not
bool OE_Tester::hasCrashed( double* q ){
  double norm = sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );
  // we will consider a crash in the algorithm if:
  // - q is not of unit norm
  // - q is a NaN
  
  return !(1.1 > norm);
}



void OE_Tester::printMatrix( double* M , int n , int m ){
  for(int i=0; i<n; i++){
    for(int j=0; j<m; j++) std::cout << std::setw(10) << std::setprecision(10) << " " << M[i+j*n];
    std::cout << "\n";
  }
  std::cout << "\n";
  
  return;
}



