function [Rob, Sen, Lmk, Trj, Frm, Fac, factorRob, Map_final] = ...
    propagate_uncertainty(points_control, ...
    Rob, SimLmk, Sen, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt)
%PROPAGATE_UNCERTAINTY Simulate covariance propagation along a candidate
%path.

global Map

current_frame = 1;

while ~isempty(points_control)
    
    % Generate control commands.
    u = points_control(1,:) - Rob.state.x(1:3)';
    Rob.con.u(1:3) = u;
    % Pop target control point from queue.
    points_control = points_control(2:end,:);
    
    % Simulate control for this time-step.
    Rob.con.u = ...
        Rob.con.u + Rob.con.uStd.*randn(size(Rob.con.uStd));
    Raw = simObservation(Rob, Sen, SimLmk, Opt);
    Rob = simMotion(Rob,[]);
    % Integrate odometry for relative motion factors.
    factorRob.con.u = Rob.con.u;
    factorRob = integrateMotion(factorRob, []);
    
    if (mod(current_frame, Opt.map.kfrmPeriod) == 0) || ...
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
    
    current_frame = current_frame + 1;
    
end

Map_final = Map;

end

