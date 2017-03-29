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
 * File:   main.cpp
 * Author: P.Bernal-Polo
 * 
 * Created on February 19, 2017, 2:05 PM
 */


#include "config.h"

#include <cstdlib>

#include "MEKFcO.h"
#include "MUKFcO.h"
#include "MEKFcRP.h"
#include "MUKFcRP.h"
#include "MEKFcMRP.h"
#include "MUKFcMRP.h"
#include "MEKFcRV.h"
#include "MUKFcRV.h"
#include "MadgwickAHRS.h"

#include "OE_Tester.h"

//using namespace std;
#include <math.h>

/*
 * 
 */
int main(int argc, char** argv) {
  
  
  int Nestimators = 0;
  OrientationEstimator *estimator[9];
  std::string titles[9];
  
#if defined MUKF_O
  estimator[Nestimators] = new MUKFcO();
  titles[Nestimators] = "MUKFcO";
  Nestimators++;
#endif
#if defined MUKF_RP
  estimator[Nestimators] = new MUKFcRP();
  titles[Nestimators] = "MUKFcRP";
  Nestimators++;
#endif
#if defined MUKF_MRP
  estimator[Nestimators] = new MUKFcMRP();
  titles[Nestimators] = "MUKFcMRP";
  Nestimators++;
#endif
#if defined MUKF_RV
  estimator[Nestimators] = new MUKFcRV();
  titles[Nestimators] = "MUKFcRV";
  Nestimators++;
#endif
#if defined MEKF_O
  estimator[Nestimators] = new MEKFcO();
  titles[Nestimators] = "MEKFcO";
  Nestimators++;
#endif
#if defined MEKF_RP
  estimator[Nestimators] = new MEKFcRP();
  titles[Nestimators] = "MEKFcRP";
  Nestimators++;
#endif
#if defined MEKF_MRP
  estimator[Nestimators] = new MEKFcMRP();
  titles[Nestimators] = "MEKFcMRP";
  Nestimators++;
#endif
#if defined MEKF_RV
  estimator[Nestimators] = new MEKFcRV();
  titles[Nestimators] = "MEKFcRV";
  Nestimators++;
#endif
#if defined Madgwick_AHRS
  estimator[Nestimators] = new Madgwick();
  titles[Nestimators] = "Madgwick";
  Nestimators++;
#endif
  
  
  
  double R = 1.0;
  for(int k=0; k<3; k++){
    R /= 100.0;
    
    
    std::string path = "./output/";
    std::string aftertitle = "";
#if defined CHART_UPDATE
    aftertitle += "_cu_R=";
#else
    aftertitle += "_ncu_R=";
#endif
    
#if defined R1e_2
    double R = 1.0e-2;
#elif defined R1e_4
    double R = 1.0e-4;
#elif defined R1e_6
    double R = 1.0e-6;
#endif
    
    
    aftertitle += std::to_string( R );
    
    
    for(int n=0; n<Nestimators; n++) {
      
      try{
        
        std::cout << titles[n] << " " << R << "\n";
        
        OE_Tester myTester;
        myTester.set_title(path + titles[n] + aftertitle);
        const int Nfrequencies = 7;
        double f[Nfrequencies] = {1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0};
        myTester.set_f(f, Nfrequencies);
        myTester.set_convergenceThreshold(0.1);
        myTester.set_maxConvergenceUpdates(10000);
        myTester.set_Tsim(5.0);
        myTester.set_simStepsPerUpdate(20);
        myTester.set_Ntimes(10000);
        myTester.set_Qw(1.0e0);
        myTester.set_Qa(1.0e-2);
        myTester.set_Rw(R);
        myTester.set_Ra(R);
        
        //myTester.test( &estimator , 2.0 );
        myTester.multithread_test(estimator[n], 50.0, 8);
        
      }catch( int e ){
        return 1;
      }
      
    }
    
  }
  
  // we release the memory
  for(int n=0; n<Nestimators; n++) delete estimator[n];
  
  return 0;
}

