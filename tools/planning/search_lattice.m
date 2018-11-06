function path = search_lattice(Rob_init, lattice, field_map, ...
    SimRob, SimSen, SimLmk, SimOpt, Sen, Lmk, Obs, Trj, Frm, Fac, factorRob, Tim, Opt, ...
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

lattice = [5,5,5];

global Map

%% EKF (7-pose) approach.
% Read robot covariance from the graphSLAM map.
%v_pose = qpose2vpose(Rob_init.state.x); % (7-pose -> 6-pose) Transform current robot pose.
%[~, V_x] = vpose2qpose(v_pose);         % (6-pose -> 7-pose) Get Jacobian.
%r = Rob_init.state.r;
%Rob_init.state.P = V_x*Map.P(r,r)*V_x'; % (6-pose -> 7-pose) Transform covariance from graphSLAM map result.
%% Graph-based (6-pose) approach.
r = Rob_init.state.r(1:3);
Rob_P = Map.P(r,r);

P_trace_prev = sum(field_map.cov);
point_prev = Rob_init.state.x(1:3)';
path = Rob_init.state.x(1:3)';
Rob_prev = Rob_init;

% num_lattice_points = size(lattice,1);
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
        disp(Rob_P)
        
        % Simulate covariance propagation assuming constant velocity.
        if ~isempty(points_control)
            
            % Generate control commands.
            u = points_control(1,:) - Rob_eval.state.x(1:3)';
            
            %% EKF (7-pose) approach.
            % % 1. Motion
            % Rob_eval.con.u = ...
            % [u';0;0;0] + Rob_eval.con.uStd.*randn(size(Rob_eval.con.uStd));
            % % Propagate motion uncertainty.
            % Rob_eval = integrateMotion(Rob_eval, Tim);
            %
            % % 2. Observation.
            % % Observe simulated landmarks using sensor.
            %  Raw = simObservation(SimRob, SimSen, SimLmk, SimOpt) ;
            
            % Predict EKF update for known landmarks.
            % [Rob_eval, Sen, Lmk, Obs] = ...
            %    predictKnownLmkCorr(Rob_eval, Sen, Raw, Lmk, Obs, Frm, Opt);
            
            %% Graph-based (6-pose) approach.
            % Observe simulated landmarks using sensor.
            Raw = simObservation(SimRob, SimSen, SimLmk, SimOpt);
            
            % Simulate control for this time-step.
            Rob_eval.con.u = ...
                [u';0;0;0] + Rob_eval.con.uStd.*randn(size(Rob_eval.con.uStd));
            Rob_eval = simMotion(Rob_eval,Tim);
            % Integrate odometry for relative motion factors.
            factorRob.con.u = Rob_eval.con.u;
            factorRob = integrateMotion(factorRob, Tim);
            
            % Pop target control point from queue.
            points_control = points_control(2:end,:);
            
        end
        
        % Add motion factor - odometry constraint.
        [Rob_eval, Lmk, Trj, Frm, Fac] = ...
            addKeyFrame(Rob_eval, Lmk, Trj ,Frm, Fac, factorRob, 'motion');
        
        % Add measurement factors - observation constraints.
        % Observe known landmarks.
        [Rob_eval, Sen, Lmk, Obs, Frm(1,Trj.head), Fac] = ...
            addKnownLmkFactors(Rob_eval, Sen, Raw, ...
            Lmk, Obs, Frm(:,Trj.head), Fac, Opt);
        
        % graphSLAM optimisation.
        % Set-up and solve the  problem.
        [Rob_eval, Sen, Lmk, Obs, Frm, Fac] = ...
            solveGraph(Rob_eval, Sen, Lmk, Obs, Frm, Fac, Opt);
        
        disp('Final pose cov.: ')
        r = Rob_eval.state.r(1:3);
        Rob_P = Map.P(r,r);
        disp(Rob_P)
        
        field_map_eval = predict_map_update(point_eval, Rob_P, field_map, ...
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