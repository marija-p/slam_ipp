function [] = debug_ol_control_noise(control_points, Rob, planning_params)

SimRob = Rob;
factorRob = resetMotion(Rob);

%% Path generation.
% Create polynomial path through the control points.
trajectory = ...
    plan_path_waypoints(control_points, planning_params.max_vel, planning_params.max_acc);

% Sample trajectory to find locations to take measurements at.
[~, points_control, ~, ~] = ...
    sample_trajectory(trajectory, 1/planning_params.control_freq);

%% Control/measurement simulation.
while ~isempty(points_control)
    
    % Generate control commands.
    u = points_control(1,:) - Rob.state.x(1:3)';
    Rob.con.u(1:3) = u;
    SimRob.con.u(1:3) = u;
    % Pop target control point from queue.
    points_control = points_control(2:end,:);
    
    % Simulate control for this time-step.
    du = normrnd(0, abs(Rob.con.u(1:3))'.*planning_params.control_noise_coeffs)';
    Rob.con.u(1:3) = Rob.con.u(1:3) + du;
    Rob.con.U(1:3,1:3) = diag(max([1e-8; 1e-8; 1e-8], du.^2));
    
    SimRob = simMotion(SimRob,[]);
    Rob = simMotion(Rob,[]);
    
    % Integrate odometry for relative motion factors.
    factorRob.con.u = Rob.con.u;
    factorRob.con.U = Rob.con.U;
    factorRob = integrateMotion(factorRob, []);
    
    % Add keyframes and solve graphSLAM.
%     if (mod(current_frame, Opt.map.kfrmPeriod*planning_params.keyframe_predict_factor) == 0) || ...
%             isempty(points_control)
%         
%         % Add motion factor - odometry constraint.
%         [Rob, Lmk, Trj, Frm, Fac] = ...
%             addKeyFrame(Rob, Lmk, Trj, Frm, Fac, factorRob, 'motion');
%         
%         % Add measurement factors - observation constraints.
%         % Observe known landmarks.
%         [Rob, Sen, Lmk, Obs, Frm(:,Trj.head), Fac] = ...
%             addKnownLmkFactors(Rob, Sen, Raw, Lmk, Obs, ...
%             Frm(:,Trj.head), Fac, Opt);
%         
%         % graphSLAM optimisation - set-up and solve the problem.
%         [Rob, Sen, Lmk, Obs, Frm, Fac] = ...
%             solveGraph(Rob, Sen, Lmk, Obs, Frm, Fac, Opt);
%         
%         % Update robots with Frm info.
%         Rob = frm2rob(Rob, Frm(:,Trj.head));
%         % Reset motion robot
%         factorRob = resetMotion(Rob);
%         
%    end
    
%    r = Rob.state.r(1:3);
%    Rob_P_map = Map.P(r,r);
    
end

Rob_P = factorRob.state.P(1:3,1:3);

disp('Difference between real and estimated robot positions: ')
disp(abs(Rob.state.x(1:3)' - SimRob.state.x(1:3)'))
disp('Robot standard deviation: ')
disp(sqrt(diag(Rob_P)'))

end