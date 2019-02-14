obj = compute_objective_ros(path_points, yaw_init, Rob_P_init, field_map, occupancy_map, ...
    training_data, testing_data, map_params, planning_params, gp_params, transforms);
% Calculates the expected informative objective for a path.
% ---
% Inputs:
% control_points: list of waypoints to navigate to
% yaw_init: initial robot yaw [rad]
% Rob_P_init: initial robot pose covariance (3x3)
% field_map: current GP field map
% occupancy_map: gmapping occupancy grid output
% ---
% Output:
% obj: informative objective value (to be minimized)
% ---
% M Popovic 2019
%

switch planning_params.obj_func
    case {'uncertainty_adaptive', 'uncertainty_rate_adaptive', 'renyi_adaptive'}
        above_thres_ind = find(field_map.mean + ...
            planning_params.beta*sqrt(field_map.cov) >= planning_params.lower_thres);
        P_i = sum(log(field_map.cov(above_thres_ind).*exp(1)));
    case {'uncertainty', 'uncertainty_rate', 'renyi'}
        P_i = sum(log(field_map.cov.*exp(1)));
    otherwise
        warning('Unknown objective function!');
end

try
    
    [field_map, Rob_P] = predict_path_ros(path_points, yaw_init, Rob_P_init, ...
        field_map, occupancy_map, training_data, testing_data, ...
        map_params, gp_params, planning_params, transforms);
    
    switch planning_params.obj_func
        case 'uncertainty_adaptive'
            above_thres_ind = find(field_map.mean + ...
                planning_params.beta*sqrt(field_map.cov) >= planning_params.lower_thres);
            P_f = sum(log(field_map.cov(above_thres_ind).*exp(1)));
            cost = 1;
        case 'uncertainty'
            P_f = sum(log(field_map.cov.*exp(1)));
            cost = 1;
        case 'uncertainty_rate_adaptive'
            above_thres_ind = find(field_map.mean + ...
                planning_params.beta*sqrt(field_map.cov) >= planning_params.lower_thres);
            P_f = sum(log(field_map.cov(above_thres_ind).*exp(1)));
            cost = max(get_trajectory_total_time(trajectory), 1/planning_params.meas_freq);
        case 'uncertainty_rate'
            P_f = sum(log(field_map.cov.*exp(1)));
            cost = max(get_trajectory_total_time(trajectory), 1/planning_params.meas_freq);
        case 'renyi_adaptive'
            above_thres_ind = find(field_map.mean + ...
                planning_params.beta*sqrt(field_map.cov) >= planning_params.lower_thres);
            if strcmp(planning_params.renyi_uncertainty, 'Dopt')
                alpha = 1 + 1/det(Rob_P_traj);
            elseif strcmp(planning_params.renyi_uncertainty, 'Aopt')
                alpha = 1 + 1/trace(Rob_P_traj);
            end
            renyi_term = alpha^(1/(alpha-1));
            P_f = sum(log(field_map.cov(above_thres_ind).*renyi_term));
            cost = 1;
        case 'renyi'
            if strcmp(planning_params.renyi_uncertainty, 'Dopt')
                alpha = 1 + 1/det(Rob_P_traj);
            elseif strcmp(planning_params.renyi_uncertainty, 'Aopt')
                alpha = 1 + 1/trace(Rob_P_traj);
            end
            renyi_term = alpha^(1/(alpha-1));
            P_f = sum(log(field_map.cov.*renyi_term));
            cost = 1;
        otherwise
            warning('Unknown objective function!');
    end
    
    % Formulate objective.
    gain = P_i - P_f;
    obj = -gain/cost;
    
% Optimization doesn't work for some reason. xD
catch
    obj = Inf;
end