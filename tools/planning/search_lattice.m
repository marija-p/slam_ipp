function path = search_lattice(Rob_init, lattice, field_map, map_params, ...
    planning_params)
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

% [~, max_ind] = max(field_map.cov);
% [max_i, max_j, max_k] = ...
%     ind2sub([dim_y, dim_x, dim_z], max_ind);
% goal_pose = ...
%     grid_to_env_coordinates([max_j, max_i, max_k], map_params);

% Read robot covariance from graphSLAM map.
global Map
r = Rob_init.state.r;
Rob_init.state.P = Map.P(r,r);

P_trace_prev = trace(field_map.cov);
point_prev = Rob_init.state.x(1:3)';
path = Rob_init.state.x(1:3)';

% First measurement?
%grid_map = predict_map_update(point_init, grid_map, ...
%    map_parameters, planning_parameters);

while (planning_params.control_points > size(path, 1))
    
    % Initialise best solution so far.
    obj_min = Inf;
    point_best = -Inf;
    
    for i = 1:size(lattice, 1)
        
        point_eval = lattice(i, :);
        Rob_eval = Rob_init;
        
        % Create simple trajectory to candidate point assuming constant velocity.
        travel_time = pdist([point_prev; point_eval])/planning_params.max_vel);
        num_points = travel_time*planning_params.control_freq;
        points_control = ...
            [linspace(point_prev(1),point_eval(1), num_points)', ...
            linspace(point_prev(2),point_eval(2), num_points)', ...
            linspace(point_prev(3),point_eval(3), num_points)'];
        
        % Simulate covariance propagation assuming constant velocity.
        if ~isempty(points_control)
            
            u = points_control(1,:) - Rob_eval.state.x(1:3)';
            Rob_eval.con.u(1:3) = u;

            Rob(rob).con.u = ...
                SimRob(rob).con.u + Rob(rob).con.uStd.*randn(size(Rob(rob).con.uStd));
            
            Rob(rob) = simMotion(Rob(rob),Tim);
            
            % Integrate odometry for relative motion factors
            factorRob(rob).con.u = Rob(rob).con.u;
            factorRob(rob) = integrateMotion(factorRob(rob),Tim);
            
            % Pop target control point from queue.
            points_control = points_control(2:end,:);
            
        end
        
        grid_map_eval = predict_map_update(point_eval, field_map, ...
            map_params, planning_params);
        P_trace = trace(grid_map_eval.P);
        
        gain = P_trace_prev - P_trace;
        cost = max(travel_time, 1/planning_params.meas_frequency);
        obj = -gain/cost;
        
        %disp(['Point ', num2str(point_eval)]);
        %disp(['Gain: ', num2str(gain)])
        %disp(['Cost: ', num2str(cost)])
        %disp(num2str(obj));
        
        % Update best solution.
        if (obj < obj_min)
            obj_min = obj;
            point_best = point_eval;
        end
        
    end
    
    % Update the map with measurement at best point.
    field_map = predict_map_update(point_best, field_map, ...
        map_params, planning_params);
    disp(['Point ', num2str(size(path,1)+1), ' at: ', num2str(point_best)]);
    disp(['Trace of P: ', num2str(trace(field_map.P))]);
    disp(['Objective: ', num2str(obj_min)]);
    path = [path; point_best];
    
    P_trace_prev = trace(field_map.P);
    point_prev = point_best;
    
end

end