classdef CompassMeasurementEdge < g2o.core.BaseUnaryEdge
   
    % Q1c:
    % This implementation contains a bug. Identify the problem
    % and fix it as per the question.

    properties(Access = protected)
        
        compassAngularOffset;
        
    end
    
    methods(Access = public)
    
        function this = CompassMeasurementEdge(compassAngularOffset)
            this = this@g2o.core.BaseUnaryEdge(1);
            this.compassAngularOffset = compassAngularOffset;
        end
        
        function computeError(this)
            x = this.edgeVertices{1}.estimate();
            %Change from:
%             this.errorZ = x(3) + this.compassAngularOffset - this.z;
            %To the following line:
            this.errorZ = this.z - x(3) - this.compassAngularOffset;
            %Wrap the heading error to -pi to pi like lab 5
            this.errorZ = g2o.stuff.normalize_theta(this.errorZ);
        end
        
        function linearizeOplus(this)
            % Change from [0 0 1] to [0 0 -1]
            this.J{1} = [0 0 -1];
        end        
    end
end