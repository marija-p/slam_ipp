function path = search_lattice(Rob_init, lattice, field_map, ...
    SimLmk, Sen_init, Lmk_init, Obs_init, Trj_init, Frm_init, Fac_init, factorRob_init, Opt, ...
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

% Testing stuff.
load('testing_data.mat')
lattice = [0, 0, 5; 5.75, 5.75, 5; 0, 0, 5];

global Map
Map_init = Map;

P_trace_prev = sum(field_map.cov);
point_prev = Rob_init.state.x(1:3)';
path = Rob_init.state.x(1:3)';

[Rob_prev, Sen_prev, Lmk_prev, Obs_prev, ...
    Trj_prev, Frm_prev, Fac_prev, factorRob_prev] = ...
    copy_graphslam_vars(Rob_init, Sen_init, Lmk_init, Obs_init, ...
    Trj_init, Frm_init, Fac_init, factorRob_init);
Map_prev = Map;

r = Rob_init.state.r(1:3);
Rob_P = Map.P(r,r);
disp('Initial pose cov.: ')
disp(Rob_P)

while (planning_params.control_points > size(path, 1))
    
    % Initialise best solution so far.
    obj_min = Inf;
    point_best = -Inf;
    
    for i = 1:size(lattice, 1)
        
        point_eval = lattice(i, :);
        [Rob_eval, Sen_eval, Lmk_eval, Obs_eval, ...
            Trj_eval, Frm_eval, Fac_eval, factorRob_eval] = ...
            copy_graphslam_vars(Rob_prev, Sen_prev, Lmk_prev, Obs_prev, ...
            Trj_prev, Frm_prev, Fac_prev, factorRob_prev);
        Map = Map_prev;
        
        % Create simple trajectory to candidate point assuming constant velocity.
        travel_time = pdist([point_prev; point_eval])/(planning_params.max_vel);
        num_points = travel_time*planning_params.control_freq;
        points_control = ...
            [linspace(point_prev(1),point_eval(1), num_points)', ...
            linspace(point_prev(2),point_eval(2), num_points)', ...
            linspace(point_prev(3),point_eval(3), num_points)'];
        
        % Propagate robot uncertainty using graphSLAM.
        [Rob_eval, Sen_eval, Lmk_eval, Trj_eval, ...
            Frm_eval, Fac_eval, factorRob_eval, Map_eval] = ...
            propagate_uncertainty(points_control, ...
            Rob_eval, Sen_eval, SimLmk, Lmk_eval, Obs_eval, ...
            Trj_eval, Frm_eval, Fac_eval, factorRob_eval, Opt);
        
        r = Rob_eval.state.r(1:3);
        Rob_P = Map_eval.P(r,r);
        disp('Final pose cov.: ')
        disp(Rob_P)
        
        field_map_eval = predict_map_update(point_eval, Rob_P, field_map, ...
            training_data, testing_data, map_params, gp_params);
        P_trace = sum(field_map_eval.cov);
        
        gain = P_trace_prev - P_trace;
        cost = max(travel_time, 1/planning_params.meas_freq);
        obj = -gain/cost;
        
        %disp(['Point: ', num2str(point_eval)]);
        %disp(['Gain: ', num2str(gain)])
        %disp(['Cost: ', num2str(cost)])
        %disp(['Objective: ', num2str(obj)]);
        
        % Update best solution.
        if (obj < obj_min)
            
            obj_min = obj;
            
            r = Rob_eval.state.r(1:3);
            Rob_P_best = Map_eval.P(r,r);
            
            point_best = point_eval;
            [Rob_best, Sen_best, Lmk_best, Obs_best, ...
                Trj_best, Frm_best, Fac_best, factorRob_best] = ...
                copy_graphslam_vars(Rob_eval, Sen_eval, Lmk_eval, Obs_eval, ...
                Trj_eval, Frm_eval, Fac_eval, factorRob_eval);
            Map_best = Map_eval;
            
        end
        
    end % end lattice candidates evaluation
    
    % Update the map with measurement at best point.
    field_map = predict_map_update(point_best, Rob_P_best, field_map, ...
        training_data, testing_data, map_params, gp_params);
    %disp(['Point ', num2str(size(path,1)+1), ' at: ', num2str(point_best)]);
    %disp(['Trace of P: ', num2str(trace(field_map.P))]);
    path = [path; point_best];
    
    P_trace_prev = sum(field_map.cov);
    point_prev = point_best;
    [Rob_prev, Sen_prev, Lmk_prev, Obs_prev, ...
        Trj_prev, Frm_prev, Fac_prev, factorRob_prev] = ...
        copy_graphslam_vars(Rob_best, Sen_best, Lmk_best, Obs_best, ...
        Trj_best, Frm_best, Fac_best, factorRob_best);
    Map_prev = Map_best;
    
end

% Important: restore global variable!
Map = Map_init;

end