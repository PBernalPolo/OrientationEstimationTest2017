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
% File:   OrientationEstimator.m
% Author: P.Bernal-Polo
% 
% Created on February 19, 2017, 2:05 PM
%


classdef (Abstract) OrientationEstimator
  %OrientationEstimator: abstract class defining the methods of an orientation estimator
  %   
  %   This class is defined to establish methods needed to test the
  %   orientation estimator in the OESimulator class
  
  properties
  end
  
  methods( Abstract )
    
    % Function: get_q
    % gets the quaternion describing the orientation
    % (rotation that transform vectors from the sensor reference frame, to the external reference frame)
    % (in case your quaternion transforms vectors from the external reference frame to the sensor reference frame, this function should return q^*)
    % inputs:
    %  obj: OrientationEstimator object that contains the quaternion
    % outputs:
    %  q: returned quaternion (q1,q2,q3,q4)=(qx,qy,qz,qw) as a column vector
    q = get_q( obj )
    
    % Function: initialize
    % this method initializes the object when we do not have information about the state
    % inputs:
    %  RwIn: covariance matrix of the angular velocity measurement
    %  RaIn: covariance matrix of the acceleration measurement
    % outputs:
    %  obj: initializated OrientationEstimator object
    obj = initialize( obj , RwIn , RaIn )
    
    % Function: updateIMU
    % method used to update the state information through an IMU measurement
    % inputs:
    %  am: measured acceleration (g)
    %  wm: measured angular velocity (rad/s)
    %  dt: time step from the last update (s)
    % outputs:
    %  obj: OrientationEstimator object with the updated state
    obj = updateIMU( obj , am , wm , dt )
    
  end
  
  
end

