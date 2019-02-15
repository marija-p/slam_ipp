function path_points = search_lattice_ros(pose_current, Rob_P_init, ...
    lattice, field_map, occupancy_map, training_data, testing_data, ...
    map_params, gp_params, planning_params, transforms)
% Performs a greedy grid search over a list of candidates to identify
% most promising points to visit based on an informative objective.
% Starting point is fixed (no measurement taken here)
% ---
% Inputs:
% point_current: starting robot point
% Rob_P_init: initial robot pose covariance (3x3)
% lattice: list of candidate points to evaluate
% field_map: current GP map (mean + covariance)
% occupancy_map: gmapping occupancy grid output
% ---
% Output:
% path_points: grid search result
% ---
% M Popovic 2018
%

dim_x = map_params.dim_x;
dim_y = map_params.dim_y;

point_current = pose_current(1:2);

% Compute initial objective.
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

point_prev = point_current;
path_points = point_current;

while (planning_params.control_points > size(path_points, 1))
    
    % Initialise best solution so far.
    obj_min = Inf;
    point_best = -Inf;
    
    %% Lattice candidate evaluation.
    for i = 1:size(lattice, 1)
        
        point_eval = lattice(i, :);
        
        % Already at this point - skip it.
        if (pdist2(point_prev, point_eval) < 0.1)
            continue;
        end
        
        % Create simple trajectory to candidate point assuming constant velocity.
        travel_time = pdist([point_prev; point_eval])/(planning_params.max_vel);
        
        % Predict robot uncertainty and predict map updates along candidate path.
        [field_map_eval, Rob_P] = ...
            predict_path_ros([point_prev; point_eval], ...
            pose_current(3), Rob_P_init, field_map, occupancy_map, ...
            training_data, testing_data, ...
            map_params, gp_params, planning_params, transforms);
        
        switch planning_params.obj_func
            case 'uncertainty_adaptive'
                above_thres_ind = find(field_map_eval.mean + ...
                    planning_params.beta*sqrt(field_map_eval.cov) >= planning_params.lower_thres);
                P_f = sum(log(field_map_eval.cov(above_thres_ind).*exp(1)));
                cost = 1;
            case 'uncertainty'
                P_f = sum(log(field_map_eval.cov.*exp(1)));
                cost = 1;
            case 'uncertainty_rate_adaptive'
                above_thres_ind = find(field_map_eval.mean + ...
                    planning_params.beta*sqrt(field_map_eval.cov) >= planning_params.lower_thres);
                P_f = sum(log(field_map_eval.cov(above_thres_ind).*exp(1)));
                cost = max(travel_time, 1/planning_params.meas_freq);
            case 'uncertainty_rate'
                P_f = sum(log(field_map_eval.cov.*exp(1)));
                cost = max(travel_time, 1/planning_params.meas_freq);
            case 'renyi_adaptive'
                above_thres_ind = find(field_map_eval.mean + ...
                    planning_params.beta*sqrt(field_map_eval.cov) >= planning_params.lower_thres);
                if strcmp(planning_params.renyi_uncertainty, 'Dopt')
                    alpha = 1 + 1/det(Rob_P);
                elseif strcmp(planning_params.renyi_uncertainty, 'Aopt')
                    alpha = 1 + 1/trace(Rob_P);
                end
                renyi_term = alpha^(1/(alpha-1));
                P_f = sum(log(field_map_eval.cov(above_thres_ind).*renyi_term));
                cost = 1;
            case 'renyi'
                if strcmp(planning_params.renyi_uncertainty, 'Dopt')
                    alpha = 1 + 1/det(Rob_P);
                elseif strcmp(planning_params.renyi_uncertainty, 'Aopt')
                    alpha = 1 + 1/trace(Rob_P);
                end
                renyi_term = alpha^(1/(alpha-1));
                P_f = sum(log(field_map_eval.cov.*renyi_term));
                cost = 1;
            otherwise
                warning('Unknown objective function!');
        end
        
        gain = P_i - P_f;
        obj = -gain/cost;
        
        %% Update best solution.
        if (obj < obj_min)
            obj_min = obj;
            point_best = point_eval;
            field_map_best = field_map_eval;
        end
        
    end
    
    %% Simulate measurement.
    % Update training data.
    training_data.X_train = [training_data.X_train; point_best];
    % Take maximum likelihood measurement based on current field map state.
    training_data.Y_train = [training_data.Y_train; ...
        interp2(reshape(testing_data.X_test(:,1),dim_y,dim_x), ...
        reshape(testing_data.X_test(:,2),dim_y,dim_x), ...
        reshape(field_map.mean,dim_y,dim_x), ...
        point_best(1), point_best(2), 'spline')];
    field_map = field_map_best;
    
    %% Write variables for next lattice evaluation.
    path_points = [path_points; point_best];
    switch planning_params.obj_func
        case {'uncertainty_adaptive', 'renyi_adaptive'}
            above_thres_ind = find(field_map.mean + ...
                planning_params.beta*sqrt(field_map.cov) >= planning_params.lower_thres);
            P_i = sum(log(field_map.cov(above_thres_ind).*exp(1)));
        case {'uncertainty', 'renyi'}
            P_i = sum(log(field_map.cov.*exp(1)));
        otherwise
            warning('Unknown objective function!');
    end
    
    point_prev = point_best;
    
    %keyboard 
    %disp('Next best point: ')
    %disp(point_best)
    
end