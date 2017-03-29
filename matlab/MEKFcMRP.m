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
% File:   MEKFcMRP.m
% Author: P.Bernal-Polo
% 
% Created on February 19, 2017, 2:05 PM
%


classdef MEKFcMRP < MEKF
  % MEKFcMRP: class implementing the Modified Rodrigues Parameters chart for the MEKF class
  %   
  %   This class implements the functions needed to perform the update of
  %   the quaternion (q) and the angular velocity (w) using the
  %   Manifold Extended Kalman Filter in the Modified Rodrigues Parameters chart
  
  
  methods(Static)
    
    % Function: fC2M
    % defines the map from the chart points, to the manifold points (through the delta quaternion)
    % inputs:
    %  e: point of the Euclidean space that we want to map to a unit quaternion
    % outputs:
    %  delta: quaternion mapped with the e point
    function delta = fC2M( e )
      % delta from the chart definition: Modified Rodrigues Parameters
      enorm = norm( e );
      if enorm > 4
        e = 4*e/enorm;
      end
      delta = [ 8*e; 16-e'*e ]/(16+e'*e);
    end
    
    
    % Function: chartUpdateMatrix
    % this function defines the transformation on the covariance matrix
    % when it is redefined from the chart centered in q quaternion, to the
    % chart centered in p quaternion, being them related by  p = q * delta
    % inputs:
    %  delta: quaternion used to update the quaternion estimation
    % outputs:
    %  G: transformation matrix to update the covariance matrix
    function G = chartUpdateMatrix( delta )
      G = 0.5*( (1+delta(4))*[ delta(4)   delta(3)  -delta(2);
                              -delta(3)   delta(4)   delta(1);
                               delta(2)  -delta(1)   delta(4)] + delta(1:3)*delta(1:3)' );
    end

  end
  
end

