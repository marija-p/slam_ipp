function path = search_lattice(Rob_init, lattice, field_map, ...
    SimLmk, Sen, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt, ...
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
lattice = [5.75, 5.75, 5];

global Map

P_trace_prev = sum(field_map.cov);
point_prev = Rob_init.state.x(1:3)';
path = Rob_init.state.x(1:3)';
Rob_prev = Rob_init;
current_frame = 1;

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
        Rob_eval = Rob_prev;
        
        % Create simple trajectory to candidate point assuming constant velocity.
        travel_time = pdist([point_prev; point_eval])/(planning_params.max_vel);
        num_points = travel_time*planning_params.control_freq;
        points_control = ...
            [linspace(point_prev(1),point_eval(1), num_points)', ...
            linspace(point_prev(2),point_eval(2), num_points)', ...
            linspace(point_prev(3),point_eval(3), num_points)'];
        
        % Simulate covariance propagation assuming constant velocity.
        while ~isempty(points_control)
            
            % Generate control commands.
            u = points_control(1,:) - Rob_eval.state.x(1:3)';
            Rob.con.u(1:3) = u;
            % Pop target control point from queue.
            points_control = points_control(2:end,:);

            % Simulate control for this time-step.
            Rob_eval.con.u = ...
                Rob.con.u + Rob_eval.con.uStd.*randn(size(Rob_eval.con.uStd));
            Raw = simObservation(Rob_eval, Sen, SimLmk, Opt);
            Rob_eval = simMotion(Rob_eval,[]);
            % Integrate odometry for relative motion factors.
            factorRob.con.u = Rob_eval.con.u;
            factorRob = integrateMotion(factorRob, []);
            
            if (mod(current_frame, Opt.map.kfrmPeriod) == 0) || ...
                    isempty(points_control)
                
                % Add motion factor - odometry constraint.
                [Rob_eval, Lmk, Trj, Frm, Fac] = ...
                    addKeyFrame(Rob_eval, Lmk, Trj, Frm, Fac, factorRob, 'motion');
                
                % Add measurement factors - observation constraints.
                % Observe known landmarks.
                [Rob_eval, Sen, Lmk, Obs, Frm(:,Trj.head), Fac] = ...
                    addKnownLmkFactors(Rob_eval, Sen, Raw, Lmk, Obs, ...
                    Frm(:,Trj.head), Fac, Opt);
                
                % graphSLAM optimisation - set-up and solve the problem.
                [Rob_eval, Sen, Lmk, Obs, Frm, Fac] = ...
                    solveGraph(Rob_eval, Sen, Lmk, Obs, Frm, Fac, Opt);
                
                % Update robots with Frm info.
                Rob_eval = frm2rob(Rob_eval, Frm(:,Trj.head));                
                % Reset motion robot
                factorRob = resetMotion(Rob_eval);
                
            end
            
            current_frame = current_frame + 1;
            
        end
        
        r = Rob_eval.state.r(1:3);
        Rob_P = Map.P(r,r);
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