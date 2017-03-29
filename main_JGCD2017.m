%
% Copyright (C) 2017 P.Bernal-Polo
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%

% 
% File:   main_JGCD2017.m
% Author: P.Bernal-Polo
% 
% Created on February 19, 2017, 2:05 PM
%

clear

% we select the algorithms to test
elections = [ 1 2 3 4 5 6 7 8 9 ];


for election = elections
  fprintf( '(algorithm %d)\n' , election )
  switch election
    case 1
      title = 'MUKFcO';
      estimator = MUKFcO();
    
    case 2
      title = 'MUKFcRP';
      estimator = MUKFcRP();
    
    case 3
      title = 'MUKFcMRP';
      estimator = MUKFcMRP();
    
    case 4
      title = 'MUKFcRV';
      estimator = MUKFcRV();
    
    case 5
      title = 'MEKFcO';
      estimator = MEKFcO();
    
    case 6
      title = 'MEKFcRP';
      estimator = MEKFcRP();
    
    case 7
      title = 'MEKFcMRP';
      estimator = MEKFcMRP();
    
    case 8
      title = 'MEKFcRV';
      estimator = MEKFcRV();
    
    case 9
      title = 'MadgwickAHRS';
      estimator = MadgwickAHRS();
    
    otherwise
      error('Not a possible value for variable "election".');
    
  end
  
  % after choosing the algorithm we set up the test parameters
  oeTester = OE_Tester();
  oeTester = oeTester.set_title( title );
  oeTester = oeTester.set_f( [1 5 10 100] );
  oeTester = oeTester.set_convergenceThreshold( 3.0 );
  oeTester = oeTester.set_maxConvergenceUpdates( 1000 );
  oeTester = oeTester.set_Tsim( 1.0 );
  oeTester = oeTester.set_simStepsPerUpdate( 10 );
  oeTester = oeTester.set_Ntimes( 10 );
  oeTester = oeTester.set_Qw( 1e0 );
  oeTester = oeTester.set_Qa( 1e-2 );
  oeTester = oeTester.set_Rw( 1e-4 );
  oeTester = oeTester.set_Ra( 1e-4 );
  
  % we can do a normal test
  % oeTester.test( estimator , 2 );
  % but it will be faster to use multiple threads
  oeTester.multithread_test( estimator , 2 , 4 );
  
end


%% finally, we can see the results of our tests

% the first and second figures ar the number of crashes, and the number of no-convergences
for n=2:3
  
  fig = figure;
  
  load MUKFcO.dat
  plot( MUKFcO(:,1) , MUKFcO(:,n) , 'b-.' )
  
  ax = get(fig,'CurrentAxes');
  set(ax,'XScale','log')
  
  hold on
  
  load MUKFcRP.dat
  plot( MUKFcRP(:,1) , MUKFcRP(:,n) , 'r-.' )
  
  load MUKFcMRP.dat
  plot( MUKFcMRP(:,1) , MUKFcMRP(:,n) , 'c-.' )
  
  load MUKFcRV.dat
  plot( MUKFcRV(:,1) , MUKFcRV(:,n) , 'g-.' )
  
  load MEKFcO.dat
  plot( MEKFcO(:,1) , MEKFcO(:,n) , 'b--' )
  
  load MEKFcRP.dat
  plot( MEKFcRP(:,1) , MEKFcRP(:,n) , 'r--' )
  
  load MEKFcMRP.dat
  plot( MEKFcMRP(:,1) , MEKFcMRP(:,n) , 'c--' )
  
  load MEKFcRV.dat
  plot( MEKFcRV(:,1) , MEKFcRV(:,n) , 'g--' )
  
  load MadgwickAHRS.dat
  plot( MadgwickAHRS(:,1) , MadgwickAHRS(:,n) , 'black-' )
  
  clear title xlabel ylabel
  if n == 2
    title('Algorithm crashes')
    ylabel('Number of algorithm crashes')
  else
    title('Non-convergent simulations')
    ylabel('Number of non-convergent simulations')
  end
  xlabel('frequency (Hz)')
  legend( 'MUKFcP' , 'MUKFcRP' , 'MUKFcMRP' , 'MUKFcRV' , 'MEKFcP' , 'MEKFcRP' , 'MEKFcMRP' , 'MEKFcRV' , 'MadgwickAHRS' )
  
  hold off
  
end

% after the first figures, come the convergence time, the error in gravity
% estimation and the error in orientation estimation

% choose the error you want to see
%nsigma = 1;  % if we choose this we will see the standard deviation of the distribution of errors
nsigma = 2;  % if we choose this we will see the standard deviation of the mean error computation

for n=1:3
  fig = figure;
  
  load MUKFcO.dat
  errorbar( MUKFcO(:,1) , MUKFcO(:,3*n+1) , MUKFcO(:,3*n+1+nsigma) , 'b-.' )
  
  ax = get(fig,'CurrentAxes');
  set(ax,'XScale','log')
  
  hold on
  
  load MUKFcRP.dat
  errorbar( MUKFcRP(:,1) , MUKFcRP(:,3*n+1) , MUKFcRP(:,3*n+1+nsigma) , 'r-.' )
  
  load MUKFcMRP.dat
  errorbar( MUKFcMRP(:,1) , MUKFcMRP(:,3*n+1) , MUKFcMRP(:,3*n+1+nsigma) , 'c-.' )
  
  load MUKFcRV.dat
  errorbar( MUKFcRV(:,1) , MUKFcRV(:,3*n+1) , MUKFcRV(:,3*n+1+nsigma) , 'g-.' )
  
  load MEKFcO.dat
  errorbar( MEKFcO(:,1) , MEKFcO(:,3*n+1) , MEKFcO(:,3*n+1+nsigma) , 'b--' )
  
  load MEKFcRP.dat
  errorbar( MEKFcRP(:,1) , MEKFcRP(:,3*n+1) , MEKFcRP(:,3*n+1+nsigma) , 'r--' )
  
  load MEKFcMRP.dat
  errorbar( MEKFcMRP(:,1) , MEKFcMRP(:,3*n+1) , MEKFcMRP(:,3*n+1+nsigma) , 'c--' )
  
  load MEKFcRV.dat
  errorbar( MEKFcRV(:,1) , MEKFcRV(:,3*n+1) , MEKFcRV(:,3*n+1+nsigma) , 'g--' )
  
  load MadgwickAHRS.dat
  errorbar( MadgwickAHRS(:,1) , MadgwickAHRS(:,3*n+1) , MadgwickAHRS(:,3*n+1+nsigma) , 'black-' )
  
  clear title xlabel ylabel
  if n == 1
    title('Convergence time')
    ylabel('convergence time (s)')
  elseif n == 2
    title('Error in gravity estimation')
    ylabel('error in gravity estimation (degrees)')
  elseif n == 3
    title('Error in orientation estimation')
    ylabel('error in orientation estimation (degrees)')
  end
  xlabel('frequency (Hz)')
  legend( 'MUKFcO' , 'MUKFcRP' , 'MUKFcMRP' , 'MUKFcRV' , 'MEKFcO' , 'MEKFcRP' , 'MEKFcMRP' , 'MEKFcRV' , 'MadgwickAHRS' )
  
  hold off
  
end



