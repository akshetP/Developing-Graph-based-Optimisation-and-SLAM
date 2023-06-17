% 3B
% This class inherit the given drivebot.DriveBotSLAMSystem to implement
% the graph pruning for Q3B

% This class is only been use for question 3B
classdef DriveBotSLAMSystem3B < drivebot.DriveBotSLAMSystem
     
    properties(Access = protected)
       % an integer variable that specify the limit of the observation
       % edge. If we are running METHOD 1 from q3_b.m, we prune the land
       % mark observation edge before the running the optimzation step
       % based on this number.
       num_limit_observation_edge 

       % an integer variable that specify which graph pruning approach we are running 
       approach
    end
    
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = DriveBotSLAMSystem3B(configuration,num_limit_observation_edge,approach)
            % Call the base class constructor to initialize the DriveBotSLAM system
            this = this@drivebot.DriveBotSLAMSystem(configuration);
            this.num_limit_observation_edge = num_limit_observation_edge;
            this.approach = approach;
        end
        
        % We overload the optimize method so that you can add additional
        % logic here
        % IF YOU ARE A RECRUITER WHO NEEDS TO RUN THIS CODE, PLEASE EMAIL ME AT
        % akshetp.ap@gmail.com

        % This function implements the first graph pruning approach which
        % we mentioned in our report
        % From Q2C, we found the average number of observations received by each landmark. 
        % In our first experinment, we try to prune all the observations
        % edge after reaching that number. For each landmark, we only keep
        % the first 460 observation edges.
        function deleteObservationEdges(this)
            fprintf("==Deleting Observation Edges using method 1 ==");
             %get all landmark vertices
             allVertices = this.graph.vertices();
             landmarkVerticesMask = cellfun('isclass',allVertices,"drivebot.graph.LandmarkStateVertex");
             allLandmarks = allVertices(landmarkVerticesMask);
             numLandmarks = sum(landmarkVerticesMask);

             % For each landmark, we get all the observation edges and
             % execute the pruning
             for i = 1:numLandmarks
                 landmarkObservationsEdges = allLandmarks{i}.edges;
                 numberOfObservationsEdges = length(landmarkObservationsEdges);
                 %If the number of observation received by a single
                 %landmark exceed the limit, remove all the observation
                 %edge after the limit
                 % IF YOU ARE A RECRUITER WHO NEEDS TO RUN THIS CODE, PLEASE EMAIL ME AT
                 % akshetp.ap@gmail.com
             end
        end

        % This function implements the second graph pruning approach.
        % In our second experinment, we are trying to reduce the number of
        % observation edge by removing 1 in every 2 vehicle vertices. 
        % For every 2 vehicle vertices, we remove the 1 landmark observation edge from the
        % graph. 
        function deleteObservationEdgesMethod2(this)
            fprintf("==Deleting Observation Edges using method 2 ==");
           
            %get all landmark vertices
            allVertices = this.graph.vertices();
            vehicleVerticesMask = cellfun('isclass',allVertices,"drivebot.graph.VehicleStateVertex");            
            allVehicleVertices = allVertices(vehicleVerticesMask);
            
            % for each landmark, we remove 1 observation edge in every 2
            % observation edge. 
            % IF YOU ARE A RECRUITER WHO NEEDS TO RUN THIS CODE, PLEASE EMAIL ME AT
            % akshetp.ap@gmail.com
        end
    end
end
