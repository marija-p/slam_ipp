function path_points = search_lattice(Rob_init, lattice, field_map, ...
    SimLmk, Sen_init, Lmk_init, Obs, Trj_init, Frm_init, Fac_init, factorRob_init, Opt, ...
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
% path_points: grid search result
% ---
% M Popovic 2018
%

%% Testing stuff.
%load('testing_data.mat')
%gp_params.N_gauss = 5;
%planning_params.control_points = 3;
%lattice = [0, 0, 5; 5.75, 5.75, 5];

dim_x = map_params.dim_x;
dim_y = map_params.dim_y;
dim_z = map_params.dim_z;

% Important: remember global variable to restore later.
global Map
Map_init = Map;

%% Prepare variables.
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

point_prev = Rob_init.state.x(1:3)';
path_points = Rob_init.state.x(1:3)';

[Rob_prev, Sen_prev, Lmk_prev, Trj_prev, Frm_prev, Fac_prev, factorRob_prev] = ...
    copy_graphslam_vars(Rob_init, Sen_init, Lmk_init, Trj_init, Frm_init, Fac_init, factorRob_init);
Map_prev = Map;

%r = Rob_init.state.r(1:3);
%Rob_P = Map.P(r,r);
%disp('Initial pose cov.: ')
%disp(Rob_P)

while (planning_params.control_points > size(path_points, 1))
    
    % Initialise best solution so far.
    obj_min = Inf;
    point_best = -Inf;
    
    %% Lattice candidate evaluation.
    for i = 1:size(lattice, 1)
        
        point_eval = lattice(i, :);
        [Rob_eval, Sen_eval, Lmk_eval, Trj_eval, Frm_eval, Fac_eval, factorRob_eval] = ...
            copy_graphslam_vars(Rob_prev, Sen_prev, Lmk_prev, Trj_prev, Frm_prev, Fac_prev, factorRob_prev);
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
            Rob_eval, Sen_eval, SimLmk, Lmk_eval, Obs, ...
            Trj_eval, Frm_eval, Fac_eval, factorRob_eval, Opt, ...
            planning_params);
        r = Rob_eval.state.r(1:3);
        Rob_P = Map_eval.P(r,r) + factorRob_eval.state.P(1:3,1:3);
        
        % Predict map update at target location.
        field_map_eval = predict_map_update(point_eval, Rob_P, field_map, ...
            training_data, testing_data, map_params, gp_params);
        
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
        
        %disp(['Point: ', num2str(point_eval)]);
        %disp(['Gain: ', num2str(gain), '. Cost: ', num2str(cost), '.'])
        %disp(['Objective: ', num2str(obj)]);
        %disp('Final pose cov.: ')
        %disp(Rob_P)
        %disp('-----------')
        
        %% Update best solution.
        if (obj < obj_min)
            obj_min = obj;
            point_best = point_eval;
            [Rob_best, Sen_best, Lmk_best, Trj_best, Frm_best, Fac_best, factorRob_best] = ...
                copy_graphslam_vars(Rob_eval, Sen_eval, Lmk_eval, Trj_eval, Frm_eval, Fac_eval, factorRob_eval);
            Map_best = Map_eval;
            field_map_best = field_map_eval;
        end
        
    end
    
    %% Simulate measurement.
    % Update training data.
    training_data.X_train = [training_data.X_train; point_best];
    % Take maximum likelihood measurement based on current field map state.
    training_data.Y_train = [training_data.Y_train; ...
        interp3(reshape(testing_data.X_test(:,1),dim_y,dim_x,dim_z), ...
        reshape(testing_data.X_test(:,2),dim_y,dim_x,dim_z), ...
        reshape(testing_data.X_test(:,3),dim_y,dim_x,dim_z), ...
        reshape(field_map.mean,dim_y,dim_x,dim_z), ...
        point_best(1), point_best(2), point_best(3), 'spline')];
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
    [Rob_prev, Sen_prev, Lmk_prev, Trj_prev, Frm_prev, Fac_prev, factorRob_prev] = ...
        copy_graphslam_vars(Rob_best, Sen_best, Lmk_best, Trj_best, Frm_best, Fac_best, factorRob_best);
    Map_prev = Map_best;
    
end

% Important: restore global variable!
Map = Map_init;

end