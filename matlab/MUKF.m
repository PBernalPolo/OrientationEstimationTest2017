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
% File:   MUKF.m
% Author: P.Bernal-Polo
% 
% Created on February 19, 2017, 2:05 PM
%


classdef (Abstract) MUKF < OrientationEstimator
  % MUKF: abstract class implementing the UKF for update the quaternion and the angular velocity
  %   
  %   This class implements most of the methods needed to perform the
  %   update of the quaternion describing the orientation (q) and the
  %   angular velocity (w) using the Manifold Unscented Kalman Filter. This
  %   class is abstract because a chart definition is needed in order to
  %   establish the full functionality
  
  properties( Access = protected )
    % quaternion used for the last update (q1,q2,q3,q4)=(qx,qy,qz,qw)
    % (rotation that transform vectors from the sensor reference frame, to the external reference frame)
    q0 = [0; 0; 0; 1];
    % last updated point in the chart
    e = [0; 0; 0];
    % quaternion describing the orientation (q1,q2,q3,q4)=(qx,qy,qz,qw)
    % (rotation that transform vectors from the sensor reference frame, to the external reference frame)
    q = [0; 0; 0; 1];
    % angular velocity (rad/s)
    w = [0; 0; 0];
    % covariance matrix
    P = eye(6,6)*1e1;
    % covariance matrix of the angular velocity noise (rad^2/s^3)
    Qw = eye(3,3)*1e-1;
    % covariance matrix of the acceleration noise (g^2)
    Qa = eye(3,3)*1e-1;
    % covariance matrix of the angular velocity measurement noise (rad^2/s^2)
    Rw = eye(3,3)*1e-3;
    % covariance matrix of the acceleration measurement noise (g^2)
    Ra = eye(3,3)*1e-3;
    
  end
  
  
  
  % abstract methods
  methods( Abstract )
    % these methods are the chart definition
    
    % Function: fM2C
    % defines the map from the manifold points, to the chart points
    % inputs:
    %  qm: mean quaternion of the distribution (is mapped with the origin of the chart)
    %  q: quaternion that we want to map with a point in the chart
    % outputs:
    %  e: point in the chart mapped with the q quaternion
    e = fM2C( qm , q )
    
    % Function: fC2M
    % defines the map from the chart points, to the manifold points
    % inputs:
    %  qm: mean quaternion of the distribution (it is mapped with the origin of the chart)
    %  e: point of the chart that we want to map to a unit quaternion in the manifold
    % outputs:
    %  q: quaternion in the manifold mapped with the e point in the chart
    q = fC2M( qm , e )
    
  end
  
  
  
  methods
    
    % Function: get_q
    % gets the quaternion describing the orientation
    % (rotation that transform vectors from the sensor reference frame, to the external reference frame)
    % (in case your quaternion transforms vectors from the external reference frame to the sensor reference frame, this function should return q^*)
    % inputs:
    %  obj: OrientationEstimator object that contains the quaternion
    % outputs:
    %  q: returned quaternion (q1,q2,q3,q4)=(qx,qy,qz,qw) as a column vector
    function q = get_q( obj )
      q = obj.q;
    end
    
    % Function: initialize
    % this method initializes the object when we do not have information about the state
    % inputs:
    %  RwIn: covariance matrix of the angular velocity measurement
    %  RaIn: covariance matrix of the acceleration measurement
    % outputs:
    %  obj: initializated OrientationEstimator object
    function obj = initialize( obj , RwIn , RaIn )
      obj.q0 = [0; 0; 0; 1];
      obj.e = [0; 0; 0];
      obj.q = [0; 0; 0; 1];
      obj.w = [0; 0; 0];
      obj.P = 1e2*eye(6,6);
      obj.P(3,3) = 1e-16;
      obj.Qw = 1.0e0*eye(3,3);
      obj.Qa = 1.0e-2*eye(3,3);
      obj.Rw = RwIn;
      obj.Ra = RaIn;
    end
    
    % Function: updateIMU
    % method used to update the state information through an IMU measurement
    % inputs:
    %  am: measured acceleration (g)
    %  wm: measured angular velocity (rad/s)
    %  dt: time step from the last update (s)
    % outputs:
    %  obj: MUKF object with the updated state
    function obj = updateIMU( obj , am , wm , dt )
      
      % we define the extended covariance matrix
      Pe = [ obj.P        zeros(6,3)   zeros(6,3);
             zeros(3,6)   obj.Qw*dt    zeros(3,3);
             zeros(3,6)   zeros(3,3)   obj.Qa    ];
      
      % Cholesky factorization
      W0 = 1.0/25.0;   % [0,1]
      Wi = (1.0-W0)/(2*12);  % size(Pe) = 12x12
      alpha = 1.0/sqrt(2.0*Wi);
      Pe = alpha*chol(Pe);
      
      % we set up the sigma points
      X = zeros(13,25);
      Xp = zeros(13,25);
      Yp = zeros(6,25);
      % first we propagate the mean value
      X(:,1) = [ obj.q;  obj.w;  zeros(6,1) ];
      Xp(:,1) = MUKF.statePrediction( X(:,1) , dt );
      Yp(:,1) = MUKF.IMU_MeasurementPrediction( Xp(:,1) );
%obj.q0 = obj.q; obj.e = 0; % with this we can test if the Chart update is good or not
      % second we generate the sigma points from the P matrix
      for j=1:12
        % first, the +sigma point
        %  we do this because P is expressed in the q0 chart, but we need
        %  to express it in the q chart for the next time step
        X(:,j+1) = [ obj.fC2M( obj.q0 , obj.e+Pe(1:3,j) );  obj.w+Pe(4:6,j);  Pe(7:12,j) ];
        Xp(:,j+1) = MUKF.statePrediction( X(:,j+1) , dt );
        % we make sure that all quaternion are in the same semisphere
        if Xp(1:4,j+1)'*Xp(1:4,1) < 0
          Xp(1:4,j+1) = -Xp(1:4,j+1);
        end
        Yp(:,j+1) = MUKF.IMU_MeasurementPrediction( Xp(:,j+1) );
        % second, the -sigma point
        %  we do this because P is expressed in the q0 chart, but we need
        %  to express it in the q chart for the next time step
        X(:,j+13) = [ obj.fC2M( obj.q0 , obj.e-Pe(1:3,j) );  obj.w-Pe(4:6,j);  -Pe(7:12,j) ];
        Xp(:,j+13) = MUKF.statePrediction( X(:,j+13) , dt );
        % we make sure that all quaternion are in the same semisphere
        if Xp(1:4,j+13)'*Xp(1:4,1) < 0
          Xp(1:4,j+13) = -Xp(1:4,j+13);
        end
        Yp(:,j+13) = MUKF.IMU_MeasurementPrediction( Xp(:,j+13) );
      end
      
      % we compute the means
      xpm = W0*Xp(:,1) + Wi*sum(Xp(:,2:end),2);
      xpm(1:4) = xpm(1:4)/norm(xpm(1:4));
      ypm = W0*Yp(:,1) + Wi*sum(Yp(:,2:end),2);
      
      % matrices we will need
      dX = [ obj.fM2C( xpm(1:4) , Xp(1:4,1) );  Xp(5:7,1)-xpm(5:7) ];
      dY = Yp(:,1)-ypm;
      Pxx = W0*(dX*dX');
      Pxy = W0*(dX*dY');
      Pyy = W0*(dY*dY');
      % now we end calculating the matrices
      for j=2:25
        dX = [ obj.fM2C( xpm(1:4) , Xp(1:4,j) );  Xp(5:7,j)-xpm(5:7) ];
        dY = Yp(:,j)-ypm;
        Pxx = Pxx + Wi*(dX*dX');
        Pxy = Pxy + Wi*(dX*dY');
        Pyy = Pyy + Wi*(dY*dY');
      end
      % finally we add the noise (the linear part)
      Pyy = Pyy + [obj.Ra       zeros(3,3);
                   zeros(3,3)   obj.Rw    ];
      
      % now we can compute the gain ( K*Pyy = Pxy )
      K = Pxy/Pyy;
      
      % and update the state in the chart
      d = K*([am; wm]-ypm);
      
      obj.q0 = xpm(1:4);
      obj.e = d(1:3);
      
      % the updated point in the chart is mapped to a quaternion
      obj.q = obj.fC2M( xpm(1:4) , obj.e );
      % and the angular velocity is updated in the usual way
      obj.w = xpm(5:7) + d(4:6);
      
      % the covariance matrix is updated in the chart centered in obj.q0
      obj.P = Pxx - K*Pyy*K';
      
      % we avoid numerical instabilities
      obj.q = obj.q/norm(obj.q);
      obj.P = 0.5*(obj.P + obj.P');
      
      % this covariance matrix is expressed in the xpm(1:4) chart (obj.q0)
      % we will have to update it to the new obj.q chart
      % that is why we do what we do at the beginning
      
    end
    
  end
  
  
  
  % static methods
  methods(Static)
    
    % Function: statePrediction
    % this method predicts the state given the previous state, and the time increment
    % inputs:
    %  x: previous state (q,w,n,a)
    %  dt: time step
    % outputs:
    %  xp: predicted state (qp,wp,np,ap)
    function xp = statePrediction( x , dt )
      % first we predict the angular velocity
      wp = x(5:7) + x(8:10);
      % w norm computation
      wnorm = norm( wp );
      % we compute qw
      if wnorm ~= 0.0
        wdt05 = 0.5*wnorm*dt;
        swdt = sin(wdt05)/wnorm;
        qw = [ wp*swdt; cos(wdt05) ];
      else
        qw = [0; 0; 0; 1];
      end
      % we compute the predicted state (q*qw,w,n,a)
      xp = [x(4)*qw(1:3) + qw(4)*x(1:3) + cross( x(1:3) , qw(1:3) );
            x(4)*qw(4) - x(1:3)'*qw(1:3);
            wp;
            x(8:13)                                               ];
    end
    
    % Function: IMU_MeasurementPrediction
    % this method predicts the measurement given a state
    % inputs:
    %  xp: state for which the measure is to be predicted
    % outputs:
    %  yp: predicted measurement
    function yp = IMU_MeasurementPrediction( xp )
      % we build the transposed rotation matrix
      RT = [-xp(2)*xp(2)-xp(3)*xp(3)   xp(1)*xp(2)+xp(3)*xp(4)   xp(1)*xp(3)-xp(2)*xp(4);
             xp(1)*xp(2)-xp(3)*xp(4)  -xp(1)*xp(1)-xp(3)*xp(3)   xp(2)*xp(3)+xp(1)*xp(4);
             xp(1)*xp(3)+xp(2)*xp(4)   xp(2)*xp(3)-xp(1)*xp(4)  -xp(1)*xp(1)-xp(2)*xp(2)];
      RT = RT + RT + eye(3);
      % we compute the predicted measurement
      yp = [ RT*(xp(11:13)+[0; 0; 1]);  xp(5:7) ];
    end
    
  end
  
  
end

