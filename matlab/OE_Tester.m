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
% File:   OE_Tester.m
% Author: P.Bernal-Polo
% 
% Created on February 19, 2017, 2:05 PM
%


classdef OE_Tester
  % OE_Tester: class used for testing the performance of an OrientationEstimator algorithm
  %   
  %   This class implements some methods with the objective of testing, in
  %   an easy way, the performance of an algorithm of the
  %   OrientationEstimator class
  
  properties( Access = private )
    title = 'simulation';  % title used to save results
    f = [1 10 100 1000];  % vector of frequency samples (Hz)
    convergenceThreshold = 3*pi/180.0;  % threshold for which convergence is considered achieved (it is stored in radians, but introduced in degrees)
    maxConvergenceUpdates = 10000;  % maximum number of updates when trying to achieve convergence
    Tsim = 1;  % simulation time (s)
    dtdtsim = 10;  % (dt/dtsim) measure of how big is the update time step (dt) vs the simulation time step (dtsim)
    Ntimes = 100;  % times to repeat the simulation for each frequency
    Qw = 1e0;  % angular velocity variance per unit of time (rad^2/s^3)
    Qa = 1e-2;  % acceleration variance (g^2)
    Rw = 1e-4;  % variance in measured angular velocity (rad^2/s^2)
    Ra = 1e-4;  % variance in measured acceleration (g^2)
  end
  
  
  
  methods
    
    % Function: set_title
    % sets the title for files to save results
    % inputs:
    %  title: title of the file to save results
    % outputs:
    %  obj: OE_Tester object with the new title
    function obj = set_title( obj , title )
      if ~ischar(title)
        error('Error in "set_title()": title must be a string');
      end
      obj.title = title;
    end
    
    
    % Function: set_f
    % sets the frequencies for which the performance of the filter will be evaluated.
    % the frequency vector introduced here will be modified choosing the
    % nearest frequencies that produce an update at obj.Tsim
    % inputs:
    %  fin: vector of frequencies (Hz)
    % outputs:
    %  obj: OE_Tester object with the new f
    function obj = set_f( obj , fin )
      if size( fin , 1 ) > 1
        error('Error in "set_f()": please, provide the frequency vector in a row');
      end
      if sum( fin < 1.0/obj.Tsim )
        error('Error in "set_f()": all frequencies must be greater than 1.0/obj.Tsim');
      end
      obj.f = fin;
    end
    
    
    % Function: set_convergenceThreshold
    % sets the convergence threshold for the convergence step of the simulation
    % inputs:
    %  convergenceThreshold: degrees difference for which the convergence is considered achieved (degrees)
    % outputs:
    %  obj: OE_Tester object with the new convergenceThreshold
    function obj = set_convergenceThreshold( obj , convergenceThreshold )
      if convergenceThreshold < 0
        error('Error in "set_convergenceThreshold()": convergenceThreshold must be positive.');
      end
      obj.convergenceThreshold = convergenceThreshold*pi/180.0;
    end
    
    
    % Function: set_maxConvergenceUpdates
    % sets the maximum number of updates for the convergence step of the simulation
    % inputs:
    %  maxConvergenceUpdates: maximum number of updates for the convergence step
    % outputs:
    %  obj: OE_Tester object with the new maxConvergenceUpdates
    function obj = set_maxConvergenceUpdates( obj , maxConvergenceUpdates )
      if maxConvergenceUpdates < 0
        error('Error in "set_maxConvergenceUpdates()": maxConvergenceUpdates must be positive.');
      end
      obj.maxConvergenceUpdates = maxConvergenceUpdates;
    end
    
    
    % Function: set_Tsim
    % sets the time for which the simulation is run
    % inputs:
    %  Tsim: period of time for which the simulation is run (s)
    % outputs:
    %  obj: OE_Tester object with the new Tsim
    function obj = set_Tsim( obj , Tsim )
      if Tsim < 1.0/min( obj.f )
        error('Error in "set_Tsim()": Tsim cannot be less than the minimum frequency.');
      end
      if Tsim <= 0.0
        error('Error in "set_Tsim()": Tsim must be greater than 0.0');
      end
      obj.Tsim = Tsim;
    end
    
    
    % Function: set_simStepsPerUpdate
    % sets the simulation steps carried before each OrientationEstimator update
    % inputs:
    %  dtdtsimin: number of simulation steps carried before each update (that turns out to be equal to dt/dtsim)
    % outputs:
    %  obj: OE_Tester object with the new dtdtsim
    function obj = set_simStepsPerUpdate( obj , dtdtsimin )
      if ceil(dtdtsimin) ~= floor(dtdtsimin)  ||  dtdtsimin <= 0
        error('Error in "set_dtdtsim": dtdtsim must be a positive integer');
      end
      obj.dtdtsim = round( dtdtsimin );
    end

    
    % Function: set_Ntimes
    % sets the number of trajectories generated for each frequency in order of testing the OrientationEstimator
    % inputs:
    %  Ntimes: number of trajectories generated for each frequency
    % outputs:
    %  obj: OE_Tester object with the new Ntimes
    function obj = set_Ntimes( obj , Ntimes )
      if Ntimes < 1
        error('Error in "set_Ntimes": Ntimes must be 1 or greater.');
      end
      obj.Ntimes = round( Ntimes );
    end
    
    
    % Function: set_Qw
    % sets the variance for the process noise in the angular velocity
    % inputs:
    %  Qw: variance for the process noise in the angular velocity per unit of time (rad^2/s^3)
    % outputs:
    %  obj: OE_Tester object with the new Qw
    function obj = set_Qw( obj , Qw )
      if Qw < 0
        error('Error in "set_Qw()": Qw must be greater than 0.0');
      end
      obj.Qw = Qw;
    end
    
    
    % Function: set_Qa
    % sets the variance for the process noise in acceleration
    % inputs:
    %  Qa: variance for the process noise in the acceleration (g^2)
    % outputs:
    %  obj: OE_Tester object with the new Qa
    function obj = set_Qa( obj , Qa )
      if Qa < 0
        error('Error in "set_Qa()": Qa must be greater than 0.0');
      end
      obj.Qa = Qa;
    end
    
    
    % Function: set_Rw
    % sets the variance for the angular velocity measurement
    % inputs:
    %  Rw: variance of the angular velocity measurement (rad^2/s^2)
    % outputs:
    %  obj: OE_Tester object with the new Rw
    function obj = set_Rw( obj , Rw )
      if Rw < 0
        error('Error in "set_Rw()": Rw must be greater than 0.0');
      end
      obj.Rw = Rw;
    end
    
    
    % Function: set_Ra
    % sets the variance for the acceleration measurement
    % inputs:
    %  Ra: variance for the acceleration measurement (g^2)
    % outputs:
    %  obj: OE_Tester object with the new Ra
    function obj = set_Ra( obj , Ra )
      if Ra < 0
        error('Error in "set_Ra()": Ra must be greater than 0.0');
      end
      obj.Ra = Ra;
    end
    
    
    % Function: test
    % uses the OE_Tester parameters to test an OrientationEstimator, and
    % saves the results in a file
    % inputs:
    %  estimator: the OrientationEstimator object for which we want to find its performance
    %  infoUpdateTime: time step to show information about the simulation progress
    function test( obj , myEstimator , infoUpdateTime )
      if ~isnumeric(infoUpdateTime)
        error('Error in "test()": infoUpdateTime must be a number.');
      end
      % first of all, let me write about the decisions taken for this
      % simulation.
      % The first thought was to generate a sequence of states that
      % generate a smooth and continuous trajectory for the orientations.
      % Then we would take simulated measurements, and use these sequences
      % for updating our estimator. There are two major problems with this
      % approach: (keep reading)
      % - we can not assert the coincidence of the final step of the
      % simulation with the final update
      % - we can not assert the coincidence of all the estimation updates
      % for all the frequencies, with a given time step of the sequence of
      % simulated states
      % A second approach to overcome these problems is to generate a
      % different trajectory for each frequency. It is done defining an
      % integer quantity dtdtsim, that determines the simulation steps
      % performed before each estimator update (asserting the equality
      % "n*dtsim = dt", with n an integer).
      % Then, the frequencies are transformed to satisfy the equality
      % "m*dt = Tsim", with m an integer.
      % After explaining this, we can start with the code
      
      % we require the condition that the final update happens at the end of the simulation:
      %    n*dt = Tsim  =>  f = 1.0/dt = n/Tsim = round(Tsim/dt)/Tsim = round(Tsim*f)/Tsim
      fs = round(obj.Tsim*obj.f)/obj.Tsim;
      % we do not want repeated frequencies
      fs = unique( fs );
      % and now we can compute the number of frequencies
      Nfrequencies = size( fs , 2 );
      
      % we compute the sigmas of the measurement covariances
      rw = sqrt( obj.Rw );
      ra = sqrt( obj.Ra );
      
      % we will save our error definitions in these vectors
      Ncrashes = zeros( Nfrequencies , 1 );
      NnoConvergence = zeros( Nfrequencies ,1);
      TConv = zeros( Nfrequencies ,1);
      TConv2 = zeros( Nfrequencies ,1);
      errorG = zeros( Nfrequencies ,1);
      errorG2 = zeros( Nfrequencies ,1);
      errorQ = zeros( Nfrequencies ,1);
      errorQ2 = zeros( Nfrequencies ,1);
      
      % we will use this variable to display some information
      msg = '';
      
      % we take the initial time to give time estimations
      timer = tic;
      beginingTime = toc(timer);
      lastTime = beginingTime;
      
      % we do it several times to get a good performance estimation
      for n=1:obj.Ntimes
        
        % we test for each frequency
        for nf=1:Nfrequencies
          % we compute the update time step
          dt = 1.0/fs(nf);
          
          % we generate random sigmas for the process
          nw = abs( normrnd( 0 , sqrt( obj.Qw ) ) );
          na = abs( normrnd( 0 , sqrt( obj.Qa ) ) );
          
          % we take a random initial orientation, and zero angular velocity
          q0 = normrnd( 0 , 1 , [4 1]);
          q0 = q0/norm(q0);
          w0 = [0; 0; 0];
          
          % and we initialize the estimator
          RwIn = eye(3,3)*obj.Rw;
          RaIn = eye(3,3)*obj.Ra;
          myEstimator = myEstimator.initialize( RwIn , RaIn );
          
          % the first simulation part is to maintain the IMU static until
          % it reaches convergence or too many updates
          theta = 1.0e2;
          Nupdates = 0;
          crashed = false;
          while theta > obj.convergenceThreshold  &&  Nupdates < obj.maxConvergenceUpdates
            % we compute the simulated measurement
            [ am , wm ] = OE_Tester.IMU_Measurement( q0 , w0 , na , ra , rw );
            % and we update the filter with the measurements
            myEstimator = myEstimator.updateIMU( am , wm , dt );
            Nupdates = Nupdates + 1;
            % we take the estimated orientation, and we check if the algorithm has crashed
            if OE_Tester.hasCrashed( myEstimator.get_q() )
              crashed = true;
              break;
            end
            % we compute the error in the gravity estimation
            theta = OE_Tester.thetaG( q0 , myEstimator.get_q() );
          end
          % we check if there has been a crash
          if crashed
            Ncrashes(nf) = Ncrashes(nf) + 1;
            continue;
          end
          % we check if there has been convergence
          if Nupdates >= obj.maxConvergenceUpdates
            % if not, we add one to the no-convergences, and we go with the next frequency
            NnoConvergence(nf) = NnoConvergence(nf) + 1;
            continue;
          end
          
          % we go ahead with the metrics computations, and with the simulation
          TConv(nf) = TConv(nf) + Nupdates;
          TConv2(nf) = TConv2(nf) + Nupdates*Nupdates;
          
          % we save the initial estimated quaternion
          qe0 = myEstimator.get_q();
          
          % we take the initial state
          q = q0;
          w = w0;
          
          % and the second step of the simulation is to generate a random
          % orientation trajectory, and try to estimate it
          %  we need the simulation time step
          dtsim = dt/obj.dtdtsim;
          %  and the number of updates
          Nupdates = round( obj.Tsim*fs(nf) );
          %  we also need the integral of thetaG, thetaG^2, thetaQ, and thetaQ^2
          thG = 0.0;
          thG2 = 0.0;
          thQ = 0.0;
          thQ2 = 0.0;
          %  now we can start iterating
          for nup=1:Nupdates
            % first we iterate in the simulation (trajectory generation)
            for nt=1:obj.dtdtsim
              [ q , w ] = OE_Tester.trajectoryStep( q , w , dtsim , nw );
            end
            % then we simulate the measurement
            [ am , wm ] = OE_Tester.IMU_Measurement( q , w , na , ra , rw );
            % finally we update the estimator with the measurement
            myEstimator = myEstimator.updateIMU( am , wm , dt );
            % we take the estimated orientation, and we check if the algorithm has crashed
            if OE_Tester.hasCrashed( myEstimator.get_q() )
              crashed = true;
              break;
            end
            % and we add the errors
            %   angle between real and estimated gravity vector
            theta = OE_Tester.thetaG( q , myEstimator.get_q() );
            thG = thG + theta;
            thG2 = thG2 + theta*theta;
            %   angle between orientations (defined as the angle we have to rotate)
            theta = OE_Tester.thetaQ( q0 , q , qe0 , myEstimator.get_q() );
            thQ = thQ + theta;
            thQ2 = thQ2 + theta*theta;
            % we repeat this for Nupdates
          end
          % we check if there has been a crash
          if crashed
            Ncrashes(nf) = Ncrashes(nf) + 1;
            continue;
          end
          
          % now we can compute the error definitions
          dtTsim = dt/obj.Tsim;
          %   error in gravity estimation
          errorG(nf) = errorG(nf) + thG*dtTsim;
          errorG2(nf) = errorG2(nf) + thG2*dtTsim;
          %   error in orientation
          errorQ(nf) = errorQ(nf) + thQ*dtTsim;
          errorQ2(nf) = errorQ2(nf) + thQ2*dtTsim;
          
        end
        
        % if enough time has passed
        if infoUpdateTime > 0.0
          currentTime = toc(timer);
          if  currentTime-lastTime > infoUpdateTime
            % we display some information about resting time, and about the simulation progress
            eTime2end = (currentTime-beginingTime)*(obj.Ntimes-n)/n;
            eTime2endD = floor( eTime2end/86400.0 );
            eTime2endH = floor( ( eTime2end-eTime2endD*86400.0 )/3600.0 );
            eTime2endM = floor( ( eTime2end-eTime2endD*86400.0-eTime2endH*3600.0 )/60.0 );
            eTime2endS = ceil( eTime2end-eTime2endD*86400.0-eTime2endH*3600.0-eTime2endM*60.0 );
            proportion = n/obj.Ntimes;
            msg0 = [ '[' repmat( '-' , 1 , round(proportion*50.0)-1 ) '>' blanks( 50-round(proportion*50.0) ) sprintf( '] (%f) ' , proportion ) 'Estimated remaining time:' ];
            if eTime2endD > 0
              msg0 = [ msg0  sprintf( ' %d d' , eTime2endD ) ];
            end
            if eTime2endH > 0
              msg0 = [ msg0  sprintf( ' %d h' , eTime2endH ) ];
            end
            if eTime2endM > 0
              msg0 = [ msg0  sprintf( ' %d m' , eTime2endM ) ];
            end
            msg0 = [ msg0  sprintf( ' %d s\n' , eTime2endS ) ];
            fprintf( repmat( '\b' , 1 , length(msg) ) );
            fprintf( msg0 );
            msg = msg0;
            
            % we update lastTime for the next infoUpdate
            lastTime = currentTime;
          end
        end
        
      end
      
      % we save the results, but first we correct the measurements
      for nf=1:Nfrequencies
        TConv(nf) = TConv(nf)/fs(nf);
        TConv2(nf) = TConv2(nf)/( fs(nf)*fs(nf) );
        errorG(nf) = errorG(nf)*180.0/pi;
        errorG2(nf) = errorG2(nf)*180.0/pi*180.0/pi;
        errorQ(nf) = errorQ(nf)*180.0/pi;
        errorQ2(nf) = errorQ2(nf)*180.0/pi*180.0/pi;
      end
      obj.saveResults( Ncrashes , NnoConvergence , TConv , TConv2 , errorG , errorG2 , errorQ , errorQ2 );
      
      % we display the time taken
      if infoUpdateTime > 0.0
        totalTime = toc(timer);
        timeD = floor( totalTime/86400 );
        timeH = floor( (totalTime-timeD*86400)/3600 );
        timeM = floor( (totalTime-timeD*86400-timeH*3600)/60 );
        timeS = ceil( totalTime-timeD*86400-timeH*3600-timeM*60 );
        fprintf( repmat('\b', 1 , length(msg) ) );
        if timeD
          fprintf( 'Time taken: %d d %d h %d m %d s\n' , [ timeD  timeH  timeM  timeS ] );
        elseif timeH
          fprintf( 'Time taken: %d h %d m %d s\n' , [ timeH  timeM  timeS ] );
        elseif timeM
          fprintf( 'Time taken: %d m %d s\n' , [ timeM  timeS ] );
        elseif timeS
          fprintf( 'Time taken: %d s\n' , timeS );
        end
      end
      
    end
    
    
    % Function: saveResults
    % used to save results of the test method
    % inputs:
    %  Ncrashes: vector (for each frequency) with number of algorithm crashes
    %  NnoConv: vector (for each frequency) with number of non-achieved convergences at the second step of the simulation
    %  eTs: vector (for each frequency) with the sum of convergence times
    %  eT2s: vector (for each frequency) with the sum of squared convergence times
    %  eGs: vector (for each frequency) with the sum of errors in gravity estimations
    %  eG2s: vector (for each frequency) with the sum of squared errors in gravity estimations
    %  eQs: vector (for each frequency) with the sum of errors in orientation estimations
    %  eQ2s: vector (for each frequency) with the sum of squared errors in orientation estimations
    function saveResults( obj , Ncrashes , NnoConv , eTs , eT2s , eGs , eG2s , eQs , eQ2s )
      % we open the file
      fileID = fopen( strcat(obj.title,'.dat') , 'w' );
      
      % we print the information about the simulation
      fprintf( fileID , '%%title = %s;\n' , obj.title );
      fprintf( fileID , '%%convergenceThreshold = %e;\n' , obj.convergenceThreshold*180.0/pi );
      fprintf( fileID , '%%maxConvergenceUpdates = %d;\n' , obj.maxConvergenceUpdates );
      fprintf( fileID , '%%Tsim = %e;\n' , obj.Tsim );
      fprintf( fileID , '%%dtdtsim = %d;\n' , obj.dtdtsim );
      fprintf( fileID , '%%Ntimes = %d;\n' , obj.Ntimes );
      fprintf( fileID , '%%Qw = %e;\n' , obj.Qw );
      fprintf( fileID , '%%Qa = %e;\n' , obj.Qa );
      fprintf( fileID , '%%Rw = %e;\n' , obj.Rw );
      fprintf( fileID , '%%Ra = %e;\n' , obj.Ra );
      
      % now, for each frequency we save:
      fprintf( fileID , '%% frequency (Hz) | n no convergences | mean convergence time (s) | sigma convergence time (s) | sigma mean convergence time (s) | mean error in gravity (degrees) | sigma of error in gravity (degrees) | sigma mean error in gravity (degrees) | mean error in orientation (degrees) | sigma of error in orientation (degrees) | sigma mean error in orientation (degrees)\n' );
      for nf=1:length(obj.f)
        Nsamples = obj.Ntimes-NnoConv(nf);
        if Nsamples == 0
          Nsamples = 1;
        end
        % - frequency (Hz)
        fprintf( fileID , ' %e' , obj.f(nf) );
        % - number of crashes
        fprintf( fileID , ' %d' , Ncrashes(nf) );
        % - number of no convergences
        fprintf( fileID , ' %d' , NnoConv(nf) );
        % - mean of the convergence time (s)
        meT = eTs(nf)/Nsamples;
        fprintf( fileID , ' %e' , meT );
        % - sigma of the convergence time (s)
        seT = sqrt( eT2s(nf)/Nsamples - meT*meT );
        fprintf( fileID , ' %e' , seT );
        % - sigma of the computation of the mean convergence time (s)
        smeT = seT/sqrt(Nsamples);
        fprintf( fileID , ' %e' , smeT );
        % - mean of the error in gravity estimation (degrees)
        meG = eGs(nf)/Nsamples;
        fprintf( fileID , ' %e' , meG );
        % - sigma of the error in gravity estimation (degrees)
        seG = sqrt( eG2s(nf)/Nsamples - meG*meG );
        fprintf( fileID , ' %e' , seG );
        % - sigma of the mean error in gravity estimation (degrees)
        smeG = seG/sqrt(Nsamples);
        fprintf( fileID , ' %e' , smeG );
        % - mean of the error in orientation estimation (degrees)
        meQ = eQs(nf)/Nsamples;
        fprintf( fileID , ' %e' , meQ );
        % - sigma of the computation of the mean error in orientation estimation (degrees)
        seQ = sqrt( eQ2s(nf)/Nsamples - meQ*meQ );
        fprintf( fileID , ' %e' , seQ );
        % - sigma of the computation of the mean error in orientation estimation (degrees)
        smeQ = seQ/sqrt(Nsamples);
        fprintf( fileID , ' %e' , smeQ );
        fprintf( fileID , '\n' );
      end
      
      % and finally, we close the file
      fclose(fileID);
      
    end
    
    
    
    % Function: multithread_test
    % this method performs the test function in multiple threads (see the test method)
    % inputs: 
    %  estimator: the OrientationEstimator object for which we want to find its performance
    %  infoUpdateTime: time step to show information about the simulation progress
    %  Nthreads: number of threads used to process the test method
    function multithread_test( obj , estimator , infoUpdateTime , Nthreads )
      
      if Nthreads > 0
        % we create an array of OE_Tester's, and estimators
        for i=1:Nthreads
          tester( i ) = obj;
          threadEstimator( i ) = estimator;
        end
        threadTimes = floor( obj.Ntimes/Nthreads );
        % but each one have different title and Ntimes properties
        totalTimes = 0;
        for i=2:Nthreads
          tester(i) = tester(i).set_title( [ obj.title  sprintf('%d',i) ] );
          tester(i) = tester(i).set_Ntimes( threadTimes );
          totalTimes = totalTimes + threadTimes;
        end
        tester(1) = tester(1).set_title( [ obj.title  '1' ] );
        tester(1) = tester(1).set_Ntimes( obj.Ntimes-totalTimes );
        
        % we execute the test in parallel
        parfor i=1:Nthreads
          if i==1
            tester(i).test( threadEstimator(i) , infoUpdateTime );
          else
            tester(i).test( threadEstimator(i) , -1 );
          end
        end
        
        % after completing the tests, we combine the information
        %   we are interested in the next values
        fs = zeros( 1 , length(obj.f) );
        Ncrashes = zeros( length(obj.f) , 1 );
        NnoConv = zeros( length(obj.f) , 1 );
        eTs = zeros( length(obj.f) , 1 );
        eT2s = zeros( length(obj.f) , 1 );
        eGs = zeros( length(obj.f) , 1 );
        eG2s = zeros( length(obj.f) , 1 );
        eQs = zeros( length(obj.f) , 1 );
        eQ2s = zeros( length(obj.f) , 1 );
        %   for each file created by each thread, we read the corresponding values
        for i=1:Nthreads
          % we open the file
          fileID = fopen( strcat( tester(i).title , '.dat' ) , 'r' );
          % we skip lines starting by '%'
          line = '%';
          while line(1) == '%'
            line = fgets( fileID );
          end
          % for each line (frequency)
          for nf=1:length(obj.f)
            % we read the data
            row = str2num( line );
            % and we add it to the global data
            Nsamples = tester(i).Ntimes-row(2)-row(3);
            if Nsamples == 0
              Nsamples = 1;
            end
            fs(nf) = row(1);
            Ncrashes(nf) = Ncrashes(nf) + row(2);
            NnoConv(nf) = NnoConv(nf) + row(3);
            eTs(nf) = eTs(nf) + row(4)*Nsamples;
            eT2s(nf) = eT2s(nf) + (row(5)*row(5)+row(4)*row(4))*Nsamples;
            eGs(nf) = eGs(nf) + row(6)*Nsamples;
            eG2s(nf) = eG2s(nf) + (row(8)*row(8)+row(7)*row(7))*Nsamples;
            eQs(nf) = eQs(nf) + row(9)*Nsamples;
            eQ2s(nf) = eQ2s(nf) + (row(11)*row(11)+row(10)*row(10))*Nsamples;
            % and we get the new line
            line = fgets( fileID );
          end
          % we close the file
          fclose( fileID );
          % and we delete the file
          delete( strcat( tester(i).title , '.dat' ) );
        end
        % finally we store the combined values
        obj = obj.set_f( fs );
        obj.saveResults( Ncrashes , NnoConv , eTs , eT2s , eGs , eG2s , eQs , eQ2s );
        
      else
        obj.test( estimator , infoUpdateTime );
      end
      
    end
    
    
  end
  
  
  
  methods( Static )
    
    % Function: trajectoryStep
    % generates a trajectory step from a previous known state
    % inputs:
    %  q: quaternion describing the real orientation
    %  w: angular velocity measured in the sensor reference frame (rad/s)
    %  dtsim: time step of the simulation (s)
    %  nw: standard deviation of the process noise for the angular velocity (rad*s^(-3/2))
    % outputs:
    %  q: new quaternion describing the real orientation of the system (it satisfies qnew = q*qw)
    %  w: new angular velocity (rad/s)
    function [ q , w ] = trajectoryStep( q , w , dtsim , nw )
      % we compute the next angular velocity using the previous one, and the process noise
      % it is necessary to multiply by the square root to get the variance to increase linearly over time
      % (the variance after a time T does not change for different dtsim choices)
      w = w + normrnd( 0 , nw , [3 1] )*sqrt(dtsim);  
      
      % we compute the next orientation using the previous orientation, and the angular velocity
      %   w norm computation
      wnorm = norm( w );
      %   we compute qw
      if wnorm ~= 0.0
        wdt05 = 0.5*wnorm*dtsim;
        swdt = sin(wdt05)/wnorm;
        qw = [ w*swdt; cos(wdt05) ];
      else
        qw = [0; 0; 0; 1];
      end
      %   we compute the new state (q*qw,w)
      q = [q(4)*qw(1:3) + qw(4)*q(1:3) + cross( q(1:3) , qw(1:3) );
           q(4)*qw(4) - q(1:3)'*qw(1:3)];
      q = q/norm(q);  % we make sure that the quaternion is normalized
    end
    
    
    % Function: IMU_Measurement
    % generates an IMU measurement from a known state
    % inputs:
    %  q: quaternion describing the orientation of the system (transform vectors from the sensor reference frame to the external reference frame)
    %  w: real angular velocity measured in the sensor reference frame (rad/s)
    %  na: standard deviation of the process noise for the acceleration (g)
    %  ra: standard deviation of the accelerometer measurements (g)
    %  rw: standard deviation of the gyroscope measurements (rad/s)
    % outputs:
    %  am: simulated accelerometer measurement (g)
    %  wm: simulated gyroscope measurement (rad/s)
    function [ am , wm ] = IMU_Measurement( q , w , na , ra , rw )
      % we compute the transposed rotation matrix
      RT = [-q(2)*q(2)-q(3)*q(3)   q(1)*q(2)+q(3)*q(4)   q(1)*q(3)-q(2)*q(4);
             q(1)*q(2)-q(3)*q(4)  -q(1)*q(1)-q(3)*q(3)   q(2)*q(3)+q(1)*q(4);
             q(1)*q(3)+q(2)*q(4)   q(2)*q(3)-q(1)*q(4)  -q(1)*q(1)-q(2)*q(2)];
      RT = RT + RT + eye(3);
      % we generate an acceleration acting on the system
      a = normrnd( 0 , na , [3 1] );
      % we obtain the simulated measurements
      am = RT*( a + [0; 0; 1] ) + normrnd( 0 , ra , [3 1] );
      wm = w + normrnd( 0 , rw , [3 1] );
    end
    
    
    % Function: hasCrashed
    % checks if there has been a crash in the algorithm
    % inputs:
    %  q: quaternion computed with the algorithm
    % outputs:
    %  crashed: boolean variable. It will be true if it has crashed; false if not
    function crashed = hasCrashed( q )
      % we will consider a crash in the algorithm if:
      % - q is not of unit norm
      % - q is a NaN
      crashed = (norm(q) > 2.0  ||  ~isreal(q));
    end
    
    
    % Function: thetaG
    % computes the error in the gravity vector estimation
    % inputs:
    %  qReal: real quaternion describing the orientation of the system
    %  qEst: estimated quaternion describing the orientation of the system
    %  (these quaternions transform vectors from the sensor reference frame to the external reference frame)
    % outputs:
    %  theta: angle between real and estimated gravity vectors (rad)
    function theta = thetaG( qReal , qEst )
      % real gravity vector
      gReal = [qReal(1)*qReal(3)-qReal(2)*qReal(4);
               qReal(2)*qReal(3)+qReal(1)*qReal(4);
               0.5-qReal(1)*qReal(1)-qReal(2)*qReal(2)];
      gReal = gReal + gReal;
      % estimated gravity vector
      gEst = [qEst(1)*qEst(3)-qEst(2)*qEst(4);
              qEst(2)*qEst(3)+qEst(1)*qEst(4);
              0.5-qEst(1)*qEst(1)-qEst(2)*qEst(2)];
      gEst = gEst + gEst;
      gRgE = gReal'*gEst;
      if gRgE > 1.0
        gRgE = 1.0;
      end
      if gRgE < -1.0
        gRgE = -1.0;
      end
      % angle between real and estimated gravity vectors (rad)
      theta = acos( gRgE );
    end
    
    
    % Function: thetaQ
    % computes the error in the overall orientation estimation
    % inputs:
    %  qr0: initial real quaternion describing the orientation in the simulation
    %  qr: current real quaternion describing the orientation in the simulation
    %  qe0: initial estimated quaternion describing the orientation in the simulation
    %  qe: current estimated quaternion describing the orientation in the simulation
    % (these quaternions transform vectors from the sensor reference frame to the external reference frame)
    % outputs:
    %  theta: angle between real and estimated orientations (rad)
    function theta = thetaQ( qr0 , qr , qe0 , qe )
      % compute the orientation difference between the initial and final real orientations
      deltar = [qr0(4)*qr(1:3)-qr(4)*qr0(1:3)-cross(qr0(1:3),qr(1:3));
                qr0(4)*qr(4)+qr0(1:3)'*qr(1:3)];
      % compute the orientation difference between the initial and final estimated orientations
      deltae = [qe0(4)*qe(1:3)-qe(4)*qe0(1:3)-cross(qe0(1:3),qe(1:3));
                qe0(4)*qe(4)+qe0(1:3)'*qe(1:3)];
      % compute the angle between differences of real and estimated orientations (rad)
      delta0re = deltar'*deltae;
      if delta0re < 0  % the next computation only makes sense for a positive delta0re
        delta0re = -delta0re;
      end
      if delta0re > 1.0
        delta0re = 1.0;
      end
      theta = 2.0*acos(delta0re);
    end
    
  end
  
  
  
end






