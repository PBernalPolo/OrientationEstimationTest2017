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
% File:   MEKF.m
% Author: P.Bernal-Polo
% 
% Created on February 19, 2017, 2:05 PM
%


classdef (Abstract) MEKF < OrientationEstimator
  % MEKF: abstract class implementing the MEKF for updating the quaternion and the angular velocity
  %   
  %   This class implements most of the methods needed to perform the
  %   update of the quaternion describing the orientation (q) and the
  %   angular velocity (w) using the Manifold Extended Kalman Filter. This 
  %   class is abstract because a chart definition is needed in order to
  %   establish the full functionality
  
  properties( Access = protected )
    % quaternion describing the orientation (q1,q2,q3,q4)=(qx,qy,qz,qw)
    % (rotation that transform vectors from the sensor reference frame, to the external reference frame)
    q = [0; 0; 0; 1];
    % angular velocity (rad/s)
    w = [0; 0; 0];
    % covariance matrix
    P = eye(6,6)*1e1;
    % covariance matrix of the angular velocity noise (rad^2/s^3)
    Qw = eye(3,3)*1e0;
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
    
    % Function: fC2M
    % defines the map from the chart points, to the manifold points (through the delta quaternion)
    % inputs:
    %  e: point of the Euclidean space that we want to map to a unit quaternion
    % outputs:
    %  delta: quaternion mapped with the e point
    delta = fC2M( e )
    
    % Function: chartUpdateMatrix
    % this function defines the transformation on the covariance matrix
    % when it is redefined from the chart centered in q quaternion, to the
    % chart centered in p quaternion, being them related by  p = q * delta
    % inputs:
    %  delta: quaternion used to update the quaternion estimation
    % outputs:
    %  G: transformation matrix to update the covariance matrix
    G = chartUpdateMatrix( delta )
    
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
      obj.q = [0; 0; 0; 1];
      obj.w = [0; 0; 0];
      obj.P = 1e2*eye(6,6);
      obj.P(3,3) = 1e-16;  % we only assume ignorance in the x and y axes
      obj.Qw = eye(3,3)*1e0;
      obj.Qa = eye(3,3)*1e-2;
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
    %  obj: MEKF object with the updated state
    function obj = updateIMU( obj , am , wm , dt )
      
      % we compute the state prediction
      wnorm = norm(obj.w);
      if wnorm ~= 0
        wdt05 = 0.5*wnorm*dt;
        swdt = sin(wdt05)/wnorm;
        qw = [ obj.w*swdt; cos(wdt05) ];
      else
        qw = [0; 0; 0; 1];
      end
      qp = [obj.q(4)*qw(1:3) + qw(4)*obj.q(1:3) + cross( obj.q(1:3) , qw(1:3) );
            obj.q(4)*qw(4) - obj.q(1:3)'*qw(1:3) ];
      
      % we compute the covariance matrix for the state prediction
      F = [-qw(2)*qw(2)-qw(3)*qw(3)   qw(1)*qw(2)+qw(3)*qw(4)   qw(1)*qw(3)-qw(2)*qw(4);
            qw(1)*qw(2)-qw(3)*qw(4)  -qw(1)*qw(1)-qw(3)*qw(3)   qw(2)*qw(3)+qw(1)*qw(4);
            qw(1)*qw(3)+qw(2)*qw(4)   qw(2)*qw(3)-qw(1)*qw(4)  -qw(1)*qw(1)-qw(2)*qw(2)];
      
      F = F + F + eye(3);
      
      M = [F           F*dt;
           zeros(3,3)  eye(3,3)  ];
      
      obj.P = M*( obj.P + [ obj.Qw*dt*dt*dt/3   -obj.Qw*dt*dt/2;
                           -obj.Qw*dt*dt/2       obj.Qw*dt     ] )*M';
      
      % we compute the measurement prediction
      ap = [ qp(1)*qp(3)-qp(2)*qp(4);
             qp(2)*qp(3)+qp(1)*qp(4);
            -qp(1)*qp(1)-qp(2)*qp(2) ];
      
      ap = ap + ap + [0; 0; 1];
      
      % we compute the covariance matrix for the measurement prediction
      F = [  0     -ap(3)   ap(2);
            ap(3)    0     -ap(1);
           -ap(2)   ap(1)    0   ];
      
      M = [F           zeros(3,3);
           zeros(3,3)  eye(3,3)  ];
      
      S = M*obj.P*M' + [obj.Qa + obj.Ra  zeros(3,3);
                        zeros(3,3)       obj.Rw    ];
      
      % now we can compute the gain
      K = obj.P*M'/S;
      
      % and update the state in the chart
      dx = K*[am-ap; wm-obj.w];
      
      % the updated point in the chart is mapped to a quaternion
      delta = obj.fC2M( dx(1:3) );
      obj.q = [qp(4)*delta(1:3) + delta(4)*qp(1:3) + cross( qp(1:3) , delta(1:3) );
               qp(4)*delta(4) - qp(1:3)'*delta(1:3) ];
      % and the angular velocity is updated in the usual way
      obj.w = obj.w + dx(4:6);
      
      % the covariance matrix is updated in the chart centered in qp
      M = eye(6,6) - K*M;
      
      obj.P = M*obj.P;
      
      % finally we update the covariance matrix from the chart centered in
      %  qp quaternion, to the chart centered in the updated q quaternion
      F = obj.chartUpdateMatrix( delta );
      
      M = [F           zeros(3,3);
           zeros(3,3)  eye(3,3)];
      
      obj.P = M*obj.P*M';
      
      % we avoid numerical instabilities
      obj.q = obj.q/norm(obj.q);
      obj.P = 0.5*(obj.P + obj.P');
    end
    
  end
  
end

