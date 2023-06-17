classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
   
    properties(Access = protected)
        
        xyOffset;
        
    end
    
    methods(Access = public)
    
        function this = GPSMeasurementEdge(xyOffset)
            this = this@g2o.core.BaseUnaryEdge(2);
            this.xyOffset = xyOffset;
        end
        
        function computeError(this)

	    % Q1d:       
        %Get X_k = [x_k,y_k,psi_k]
        x = this.edgeVertices{1}.estimate();
        
        %Construct the M matrix. In 1d, we need a 2D obsercation
        c = cos(x(3));
        s = sin(x(3));
            
        M = [c -s;
            s c;];
            

        %compute the error
        this.errorZ = this.z - x(1:2) - M * this.xyOffset;       

        end
        
        function linearizeOplus(this)

	    % Q1d:
        % Compute the jacobian of the error function wrt to the vehicle
        % state 
        %compute sin(psi) cos(psi)
        x = this.edgeVertices{1}.estimate();
        c = cos(x(3));
        s = sin(x(3));

        % The GPS position offset (in the platform-fixed frame)
        dgx = this.xyOffset(1);
        dgy = this.xyOffset(2);

        %consturct the J matrix
        this.J{1} = zeros(2,3);
        
        this.J{1}(1,1) = -1;
        this.J{1}(1,3) = dgx*s + dgy*c;
        this.J{1}(2,2) = -1;
        this.J{1}(2,3) = dgy*s - dgx*c;
        
        end
    end
end
