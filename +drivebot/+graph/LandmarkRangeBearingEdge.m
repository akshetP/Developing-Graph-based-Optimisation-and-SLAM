% This edge encodes a 3D range bearing measurement.
%
% The measurement is in spherical polar coordinates

% Jacobian from https://github.com/petercorke/robotics-toolbox-matlab/blob/master/RangeBearingSensor.m

classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    
    methods(Access = public)
    
        function this = LandmarkRangeBearingEdge()
            this = this@g2o.core.BaseBinaryEdge(2);
        end
        
        function initialize(this)
            % Q2b:
            
            %Get vehicle state 
            veh_state=this.edgeVertices{1}.x;
            
            %The measurment is from the laser, r (range),beta (bearing). See
            %DriveBotSimulator.m line 289
            this.edgeVertices{2}.x(1)=this.z(1)*cos(this.z(2)+veh_state(3))+veh_state(1);
            this.edgeVertices{2}.x(2)=this.z(1)*sin(this.z(2)+veh_state(3))+veh_state(2);
    
        end
        
        function computeError(this)

            % Q2b:

            %Get vehicle state
            vehicle_state = this.edgeVertices{1}.estimate();
            %Get landmark state
            landmark_state= this.edgeVertices{2}.estimate();
            
            % Compute r_ik based on the landmark observation model
            dx=landmark_state(1)-vehicle_state(1);
            dy=landmark_state(2)-vehicle_state(2);
            range=sqrt(dx^2+dy^2);

            % Compute the \beta_ik from landmark observation model
            bearing=atan2(dy,dx)-vehicle_state(3);
             
            % Compute the range error
            this.errorZ(1) = range-this.z(1);

            % Compute the bearing error and wrap the result to -pi to +pi
            this.errorZ(2)= g2o.stuff.normalize_theta(bearing-this.z(2));

        end
        
        function linearizeOplus(this)
            % Q2b:
            % Compute the jacobian of the error function wrt the vehicle
            % state
            x_veh = this.edgeVertices{1}.estimate();
            x_landmark= this.edgeVertices{2}.estimate();
            dx = x_landmark(1:2)-x_veh(1:2);
            r = norm(dx);
            
            this.J{1} = ...
                [-dx(1)/r -dx(2)/r 0;
                dx(2)/r^2 -dx(1)/r^2 -1];
            this.J{2} = - this.J{1}(1:2, 1:2);
        end        
    end
end
