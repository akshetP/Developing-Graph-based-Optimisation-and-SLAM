% This class uses a slightly simpler model for the vehicle kinematics used
% in the lectures. This is the more standard built in type for estimate.
%
% The model assumes that the vehicle speed is specified in the vehicle
% frame and is then projected into the world frame. Specifically,
%
% M = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
%
% The process model has the form:
%
% x = x + M * [vx;vy;theta]
%
% where vx, vy and vtheta are the velocities.
%
% The error model 
% eTheta = 

classdef VehicleKinematicsEdge < g2o.core.BaseBinaryEdge
    
    properties(Access = protected)
        % The length of the time step
        dT;
    end
    
    methods(Access = public)
        function this = VehicleKinematicsEdge(dT)
            assert(dT >= 0);
            this = this@g2o.core.BaseBinaryEdge(3);            
            this.dT = dT;
        end
       
        function initialize(this)
            
                        
            priorX = this.edgeVertices{1}.x;

            c = cos(priorX(3));
            s = sin(priorX(3));
            
            M = this.dT * [c -s 0;
                s c 0;
                0 0 1];
            
            % Compute the posterior assming no noise
            this.edgeVertices{2}.x = this.edgeVertices{1}.x + M * this.z;

            % Wrap the heading to -pi to pi
            this.edgeVertices{2}.x(3) = g2o.stuff.normalize_theta(this.edgeVertices{2}.x(3));

        end
        
        function computeError(this)
    
            % Q1b:
            
            % Get psi_k from the vehicle state
            psi = this.edgeVertices{1}.x(3);
            %construct inverse of M
            c = cos(psi);
            s = sin(psi);
            Mi = [c s 0;
                -s c 0;
                0 0 1;];
            
            % Compute the error (v_k)
            this.errorZ = Mi*((this.edgeVertices{2}.x-this.edgeVertices{1}.x)/this.dT) - this.z;

            %Wrap the heading error to -pi to pi like lab 5
            this.errorZ(3) = g2o.stuff.normalize_theta(this.errorZ(3));

        end
        
        % Compute the Jacobians
        function linearizeOplus(this)

            % Q1b:
            % Compute the jacobian of the error function wrt the vehicle
            % state

            %modify from lab5 code

            % Get the vehicle state X_k = [x_k,y_k,psi_k]
            priorX = this.edgeVertices{1}.x;
            c = cos(priorX(3));
            s = sin(priorX(3));

            % Compute X_k+1 - X_k
            dx = this.edgeVertices{2}.x - priorX;
            Mi = [c s 0;
                -s c 0;
                0 0 1];

            % Fill in the jacobian matrix based on the equation we derived
            % in the report
            this.J{2} = Mi/this.dT;
            this.J{1}(1, 1) = - c;
            this.J{1}(1, 2) = - s;
            this.J{1}(1, 3) = -dx(1) * s + dx(2) * c;
            this.J{1}(2, 1) = s;
            this.J{1}(2, 2) = - c;
            this.J{1}(2, 3) = -dx(1) * c - dx(2) * s;
            this.J{1}(3, 3) = -1;
            
            this.J{1} = this.J{1}/this.dT;

        end
    end    
end