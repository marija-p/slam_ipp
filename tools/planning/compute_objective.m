function obj = compute_objective(control_points, field_map, ...
    Rob, Sen, SimLmk, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt, ...
    num_control_frames, training_data, testing_data, ...
    map_params, planning_params, gp_params)
% Calculates the expected informative objective for a polynomial path.
% ---
% Inputs:
% control_points: list of waypoints defining the polynomial
% field_map: current GP field map
% ---
% Output:
% obj: informative objective value (to be minimized)
% ---
% M Popovic 2018
%

global Map

dim_x = map_params.dim_x;
dim_y = map_params.dim_y;
dim_z = map_params.dim_z;
dim_x_env = map_params.dim_x*map_params.res_x;
dim_y_env = map_params.dim_y*map_params.res_y;
dim_z_env = map_params.dim_y*map_params.res_z;

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

% Number of time-frames between each measurement
meas_frame_interval = planning_params.control_freq/planning_params.meas_freq;

current_frame = 1;

%% Path generation.
% Create polynomial path through the control points.
trajectory = ...
    plan_path_waypoints(control_points, planning_params.max_vel, planning_params.max_acc);

% Sample trajectory to find locations to take measurements at.
[~, points_control, ~, ~] = ...
    sample_trajectory(trajectory, 1/planning_params.control_freq);
[~, points_meas, ~, ~] = ...
    sample_trajectory(trajectory, 1/planning_params.meas_freq);

% Discard path if it is too long.
if (size(points_meas,1) > 10)
    obj = Inf;
    return;
end

% Discard path if measurements are out of environment bounds.
if (any(points_meas(:,1) > map_params.pos_x+dim_x_env) || ...
        any(points_meas(:,2) > map_params.pos_y+dim_y_env) || ...
        any(points_meas(:,1) < map_params.pos_x) || ...
        any(points_meas(:,2) < map_params.pos_y) || ...
        any(points_meas(:,3) < map_params.pos_z) || ...
        any(points_meas(:,3) > map_params.pos_z+dim_z_env))
    obj = Inf;
    return;
end

try
    
    %% Control/measurement simulation.
    while ~isempty(points_control)
        
        % Generate control commands.
        u = points_control(1,:) - Rob.state.x(1:3)';
        Rob.con.u(1:3) = u;
        % Pop target control point from queue.
        points_control = points_control(2:end,:);
        num_control_frames = num_control_frames + 1;
        
        % Simulate control for this time-step.
        Rob.con.u(1:3) = Rob.con.u(1:3) + Rob.con.u(1:3).* ...
            (-1 + 2.*rand(3,1)).*planning_params.control_noise_percent'./100;
        Raw = simObservation(Rob, Sen, SimLmk, Opt);
        Rob = simMotion(Rob,[]);
        % Integrate odometry for relative motion factors.
        factorRob.con.u = Rob.con.u;
        factorRob = integrateMotion(factorRob, []);
        
        % Add keyframes and solve graphSLAM.
        if (mod(current_frame, Opt.map.kfrmPeriod*planning_params.keyframe_predict_factor) == 0) || ...
                isempty(points_control)
            
            % Add motion factor - odometry constraint.
            [Rob, Lmk, Trj, Frm, Fac] = ...
                addKeyFrame(Rob, Lmk, Trj, Frm, Fac, factorRob, 'motion');
            
            % Add measurement factors - observation constraints.
            % Observe known landmarks.
            [Rob, Sen, Lmk, Obs, Frm(:,Trj.head), Fac] = ...
                addKnownLmkFactors(Rob, Sen, Raw, Lmk, Obs, ...
                Frm(:,Trj.head), Fac, Opt);
            
            % graphSLAM optimisation - set-up and solve the problem.
            [Rob, Sen, Lmk, Obs, Frm, Fac] = ...
                solveGraph(Rob, Sen, Lmk, Obs, Frm, Fac, Opt);
            
            % Update robots with Frm info.
            Rob = frm2rob(Rob, Frm(:,Trj.head));
            % Reset motion robot
            factorRob = resetMotion(Rob);
            
        end
        
        % Take a measurement.
        if (mod(num_control_frames-1, meas_frame_interval) == 0)
            
            % Update training data.
            training_data.X_train = [training_data.X_train; Rob.state.x(1:3)'];
            % Take maximum likelihood measurement based on current field map state.
            training_data.Y_train = [training_data.Y_train; ...
                interp3(reshape(testing_data.X_test(:,1),dim_y,dim_x,dim_z), ...
                reshape(testing_data.X_test(:,2),dim_y,dim_x,dim_z), ...
                reshape(testing_data.X_test(:,3),dim_y,dim_x,dim_z), ...
                reshape(field_map.mean,dim_y,dim_x,dim_z), ...
                Rob.state.x(1), Rob.state.x(2), Rob.state.x(3), 'spline')];
            
        end
        
        current_frame = current_frame + 1;
        
    end
    
    r = Rob.state.r(1:3);
    Rob_P = Map.P(r,r);
    field_map = predict_map_update(Rob.state.x(1:3)', Rob_P, field_map, ...
        training_data, testing_data, map_params, gp_params);
    
    switch planning_params.obj_func
        case 'uncertainty_adaptive'
            above_thres_ind = find(field_map.mean + ...
                planning_params.beta*sqrt(field_map.cov) >= planning_params.lower_thres);
            P_f = sum(log(field_map.cov(above_thres_ind).*exp(1)));
        case 'uncertainty'
            P_f = sum(log(field_map.cov.*exp(1)));
        case 'renyi_adaptive'
            above_thres_ind = find(field_map.mean + ...
                planning_params.beta*sqrt(field_map.cov) >= planning_params.lower_thres);
            if strcmp(planning_params.renyi_uncertainty, 'Dopt')
                alpha = 1 + 1/det(Rob_P);
            elseif strcmp(planning_params.renyi_uncertainty, 'Aopt')
                alpha = 1 + 1/trace(Rob_P);
            end
            renyi_term = alpha^(1/(alpha-1));
            P_f = sum(log(field_map.cov(above_thres_ind).*renyi_term));
        case 'renyi'
            if strcmp(planning_params.renyi_uncertainty, 'Dopt')
                alpha = 1 + 1/det(Rob_P);
            elseif strcmp(planning_params.renyi_uncertainty, 'Aopt')
                alpha = 1 + 1/trace(Rob_P);
            end
            renyi_term = alpha^(1/(alpha-1));
            %disp(['Alpha = ', num2str(alpha)])
            %disp(P_i)
            %disp(num2str(sum(field_map.cov)))
            %disp(num2str(sum(field_map.cov.*(alpha^(1/(alpha-1))))))
            P_f = sum(log(field_map.cov.*renyi_term));
        otherwise
            warning('Unknown objective function!');
    end
    
    % Formulate objective.
    gain = P_i - P_f;
    cost = max(get_trajectory_total_time(trajectory), 1/planning_params.meas_freq);
    obj = -gain/cost;
    
    %disp(['Gain = ', num2str(gain)])
    %disp(['Cost = ', num2str(cost)])
    %disp(['Objective = ', num2str(obj)])
    
catch
    
    % Optimization didn't work for some reason. xD
    obj = Inf;
    return;
    
end

end