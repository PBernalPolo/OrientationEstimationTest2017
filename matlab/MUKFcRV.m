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
% File:   MUKFcRV.m
% Author: P.Bernal-Polo
% 
% Created on February 19, 2017, 2:05 PM
%


classdef MUKFcRV < MUKF
  % MUKFcRV: class implementing the Rotation Vector chart for the MUKF class
  %   
  %   This class implements the functions needed to perform the update of
  %   the quaternion (q) and the angular velocity (w) using the
  %   Manifold Unscented Kalman Filter in the Rotation Vector chart
  
  
  methods(Static)
    
    % Function: fM2C
    % defines the map from the manifold points, to the chart points
    % inputs:
    %  qm: mean quaternion of the distribution (is mapped with the origin of the chart)
    %  q: quaternion that we want to map with a point in the chart
    % outputs:
    %  e: point in the chart mapped with the q quaternion
    function e = fM2C( qm , q )
      %first we compute the delta in the manifold
      delta = [qm(4)*q(1:3) - q(4)*qm(1:3) - cross( qm(1:3) , q(1:3) );
               qm'*q];
      % e from the chart definition: Rotation Vector
      if delta(4) < 0
        delta = -delta;
      end
      dnorm = norm( delta(1:3) );
      if dnorm > 1.0
        dnorm = 1.0;
      end
      if dnorm ~= 0
        e = 2*delta(1:3)/dnorm*asin(dnorm);
      else
        e = [0; 0; 0];
      end
    end
    
    
    % Function: fC2M
    % defines the map from the chart points, to the manifold points
    % inputs:
    %  qm: mean quaternion of the distribution (it is mapped with the origin of the chart)
    %  e: point of the chart that we want to map to a unit quaternion in the manifold
    % outputs:
    %  q: quaternion in the manifold mapped with the e point in the chart
    function q = fC2M( qm , e )
      % delta from the chart definition: Rotation Vector
      enorm = norm( e );
      if enorm > pi
        e = pi*e/enorm;
        enorm = pi;
      end
      if enorm ~= 0
        delta = [ e/enorm*sin(enorm/2); cos(enorm/2) ];
      else
        delta = [0; 0; 0; 1];
      end
      % now we update in the manifold with this delta
      q = [qm(4)*delta(1:3) + delta(4)*qm(1:3) + cross( qm(1:3) , delta(1:3) );
           qm(4)*delta(4) - qm(1:3)'*delta(1:3) ];
    end
    
  end
  
end

