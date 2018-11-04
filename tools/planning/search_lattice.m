function path = search_lattice(Rob_init, lattice, field_map, ...
    Tim, SimRob, SimSen, SimLmk, SimOpt, ...
    training_data, testing_data, map_params, planning_params, gp_params)
% Performs a greedy grid search over a list of candidates to identify
% most promising points to visit based on an informative objective.
% Starting point is fixed (no measurement taken here)
% ---
% Inputs:
% Rob_init: starting robot state (graphSLAM TB struct)
% lattice: list of candidate points to evaluate
% field_map: current GP map (mean + covariance)
% ---
% Output:
% path: grid search result
% ---
% M Popovic 2018
%

% Read robot covariance from the graphSLAM map.
global Map
r = Rob_init.state.r(1:3);
Rob_init.state.P(1:3,1:3) = Map.P(r,r);

P_trace_prev = sum(field_map.cov);
point_prev = Rob_init.state.x(1:3)';
path = Rob_init.state.x(1:3)';
Rob_prev = Rob_init;

num_lattice_points = size(lattice,1);

% figure;
% scatter3(testing_data.X_test(:,1), testing_data.X_test(:,2), ...
%     testing_data.X_test(:,3), 80, field_map.cov(:));
% caxis([0 2e4]);
% title(['Tr(P) = ', num2str(P_trace_prev,'%10.3e\n')])
% colorbar
% 
% figure;

while (planning_params.control_points > size(path, 1))
    
    % Initialise best solution so far.
    obj_min = Inf;
    point_best = -Inf;
    
    for i = 1:size(lattice, 1)
        
        point_eval = lattice(i, :);
        Rob_eval = Rob_prev;
        
        % Create simple trajectory to candidate point assuming constant velocity.
        travel_time = pdist([point_prev; point_eval])/planning_params.max_vel;
        num_points = travel_time*planning_params.control_freq;
        points_control = ...
            [linspace(point_prev(1),point_eval(1), num_points)', ...
            linspace(point_prev(2),point_eval(2), num_points)', ...
            linspace(point_prev(3),point_eval(3), num_points)'];
        
        disp('Initial pose cov.: ')
        disp(Rob_eval.state.P(1:3,1:3))
        
        % Simulate covariance propagation assuming constant velocity.
        if ~isempty(points_control)
            
            % 1. Motion
            % Generate control commands.
            u = points_control(1,:) - Rob_eval.state.x(1:3)';
            Rob_eval.con.u = ...
                [u';0;0;0] + Rob_eval.con.uStd.*randn(size(Rob_eval.con.uStd));
            % Propagate motion uncertainty.
            Rob_eval = integrateMotion(Rob_eval, Tim);
            % Pop target control point from queue.
            points_control = points_control(2:end,:);
            
            % Observation.
            % Observe simulated landmarks using sensor.
            Raw = simObservation(SimRob, SimSen, SimLmk, SimOpt) ;
            
        end
        
        disp('Final pose cov.: ')
        disp(Rob_eval.state.P(1:3,1:3))
        
        field_map_eval = predict_map_update(point_eval, Rob_eval, field_map, ...
            training_data, testing_data, map_params, gp_params);
        P_trace = sum(field_map_eval.cov);
        
%         subplot(1,num_lattice_points,i)
%         scatter3(testing_data.X_test(:,1), testing_data.X_test(:,2), ...
%             testing_data.X_test(:,3), 80, field_map_eval.cov(:));
%         caxis([0 2e4]);
%         title(['Tr(P) = ', num2str(P_trace,'%10.3e\n')])
%         colorbar
        
        gain = P_trace_prev - P_trace;
        cost = max(travel_time, 1/planning_params.meas_freq);
        obj = -gain/cost;
        
        %disp(['Point: ', num2str(point_eval)]);
        disp(['Gain: ', num2str(gain)])
        disp(['Cost: ', num2str(cost)])
        disp(['Objective: ', num2str(obj)]);
        
        % Update best solution.
        if (obj < obj_min)
            obj_min = obj;
            point_best = point_eval;
            Rob_best = Rob_eval;
        end
        
    end
    
    % Update the map with measurement at best point.
    %field_map = predict_map_update(point_best, field_map, ...
    %    map_params, planning_params);
    %disp(['Point ', num2str(size(path,1)+1), ' at: ', num2str(point_best)]);
    %disp(['Trace of P: ', num2str(trace(field_map.P))]);
    %disp(['Objective: ', num2str(obj_min)]);
    path = [path; point_best];
    
    %P_trace_prev = trace(field_map.P);
    point_prev = point_best;
    Rob_prev = Rob_eval;
    
end

end