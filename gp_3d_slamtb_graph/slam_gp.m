function [metrics] = slam_gp(map_params, planning_params, opt_params, gp_params, ...
    training_data, gt_data, testing_data)

% SLAMTB_GRAPH  A graph-SLAM algorithm with simulator and graphics.
%
%   This script performs multi-robot, multi-sensor, multi-landmark 6DOF
%   graph-SLAM with simulation and graphics capabilities.
%
%   Please read slamToolbox.pdf and courseSLAM.pdf in the root directory
%   thoroughly before using this toolbox.
%
%   - Beginners should not modify this file, just edit USERDATA_GRAPH.M and
%   enter and/or modify the data you wish to simulate.
%
%   See also USERDATAGRAPH, SLAMTB.
%
%   Also consult slamToolbox.pdf and courseSLAM.pdf in the root directory.

%   Created and maintained by
%   Copyright 2008, 2009, 2010 Joan Sola @ LAAS-CNRS.
%   Copyright 2011, 2012, 2013 Joan Sola.
%   Copyright 2015-     Joan Sola @ IRI-UPC-CSIC.
%   Programmers (for parts of the toolbox):
%   Copyright David Marquez and Jean-Marie Codol @ LAAS-CNRS
%   Copyright Teresa Vidal-Calleja @ ACFR.
%   See COPYING.TXT for full copyright license.

%clear
global Map

%% I. Specify user-defined options
userData_graph_gp_3d;
Robot{1}.positionStd = planning_params.position_stdev';


%% II. Initialize all data structures from user-defined data
% SLAM data
[Rob,Sen,Raw,Lmk,Obs,Trj,Frm,Fac,Tim] = createGraphStructures(...
    Robot,...
    Sensor,...
    Time,...
    Opt);

% Pre-allocate all relative-motion robots:
factorRob = Rob;

% Simulation data
[SimRob,SimSen,SimLmk,SimOpt] = createSimStructures(...
    Robot,...
    Sensor,...      % all user data
    World,...
    SimOpt);

% Mapping parameterss
dim_x = map_params.dim_x;
dim_y = map_params.dim_y;
dim_z = map_params.dim_z;

% Planning parameters
% NB: - First measurement at current robot pose
points_control = Rob.state.x(1:3)';
do_first_meas = 1;
% Number of time-frames along current trajectory
num_control_frames = 0;
% Number of time-frames between each measurement
meas_frame_interval = planning_params.control_freq/planning_params.meas_freq;
% Simulation time increment [s] = 1/control simulation frequency [Hz]
Tim.dt = 1/planning_params.control_freq;

% Lattice for 3D grid search
lattice = create_lattice(map_params, planning_params);
% GP field map
field_map = [];

% Graphics handles
if (FigOpt.drawFigs)
    [MapFig,SenFig,FldFig]          = createGraphicsStructures(...
        Rob, Sen, Lmk, Obs,...      % SLAM data
        Trj, Frm, Fac, ...
        SimRob, SimSen, SimLmk,...  % Simulator data
        testing_data.X_test, ...    % Field data
        FigOpt);                    % User-defined graphic options
    refresh_field_fig = 0;
end

% Clear user data
clear Robot Sensor World Time

%% III. Initialize data logging
metrics = initialize_metrics(map_params, planning_params, opt_params, gp_params);
metrics.landmarks = SimLmk.points.coord;

%% IV. Startup
% TODO: Possibly put in initRobots and createFrames, createFactors, createTrj...
for rob = [Rob.rob]
    
    % Reset relative motion robot
    factorRob(rob) = resetMotion(Rob(rob));
    
    % Add first keyframe with absolute factor
    Rob(rob).state.P = 1e-6 * eye(7); % Give 1cm error
    [Rob(rob),Lmk,Trj(rob),Frm(rob,:),Fac] = addKeyFrame(...
        Rob(rob),       ...
        Lmk,            ...
        Trj(rob),       ...
        Frm(rob,:),     ...
        Fac,            ...
        factorRob(rob), ...
        'absolute');
    
    for sen = Rob(rob).sensors
        
        % Initialize new landmarks
        ninits = Opt.init.nbrInits(1);
        for i = 1:ninits
            
            % Observe simulated landmarks
            Raw(sen) = simObservation(SimRob(rob), SimSen(sen), SimLmk, SimOpt);
            
            % Init new lmk
            fac = find([Fac.used] == false, 1, 'first');
            
            % Compute and allocate lmk
            [Lmk,Obs(sen,:),Frm(rob,Trj(rob).head),Fac(fac),lmk] = initNewLmk(...
                Rob(rob),   ...
                Sen(sen),   ...
                Raw(sen),   ...
                Lmk,        ...
                Obs(sen,:), ...
                Frm(rob,Trj(rob).head), ...
                Fac(fac),        ...
                Opt) ;
            
            if isempty(lmk)
                break
            end
            
        end
        
    end % end process sensors
    
    
end

%% V. Main loop
for currentFrame = Tim.firstFrame : Tim.lastFrame
    
    % Generate control commands.
    u = points_control(1,:) - Rob.state.x(1:3)';
    % For orientation
    %theta = atan2(u(2),u(1));
    SimRob.con.u(1:3) = u;
    
    % Pop target control point from queue.
    points_control = points_control(2:end,:);
    num_control_frames = num_control_frames + 1;
    
    % 1. SIMULATION
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Simulate robots
    for rob = [SimRob.rob]
        
        % Robot motion
        SimRob(rob) = simMotion(SimRob(rob),Tim);
        
        % Simulate sensor observations
        for sen = SimRob(rob).sensors
            
            % Observe simulated landmarks
            Raw(sen) = simObservation(SimRob(rob), SimSen(sen), SimLmk, SimOpt) ;
            
        end % end process sensors
        
    end % end process robots
    
    % 2. ESTIMATION
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
    %% 2.a. Robot motion prediction
    % Process robots
    for rob = [Rob.rob]
        
        % Robot motion
        Rob(rob).con.u = ...
            SimRob(rob).con.u;
        Rob(rob).con.u(1:3) = Rob(rob).con.u(1:3) + ...
            SimRob(rob).con.u(1:3).* ...
            (-1 + 2.*rand(3,1)).* ...
            planning_params.control_noise_percent'./100;
        Rob(rob) = simMotion(Rob(rob),Tim);
        
        % Integrate odometry for relative motion factors
        factorRob(rob).con.u = Rob(rob).con.u;
        factorRob(rob) = integrateMotion(factorRob(rob),Tim);
        
    end
    
    % Advance time and check if planning budget exceeded
    Map.t = Map.t + Tim.dt;
    if (Map.t > planning_params.time_budget)
        disp('Time budget exceeded!')
        return;
    end
    
    %% 2.b. Graph construction and solving
    if mod(currentFrame - Tim.firstFrame + 1, Opt.map.kfrmPeriod) == 0 || ...
            isempty(points_control)
        
        % Process robots
        for rob = [Rob.rob]
            
            % Add key frame
            [Rob(rob),Lmk,Trj(rob),Frm(rob,:),Fac] = addKeyFrame(...
                Rob(rob),       ...
                Lmk,            ...
                Trj(rob),       ...
                Frm(rob,:),     ...
                Fac,            ...
                factorRob(rob), ...
                'motion');
            
            % Process sensor observations
            for sen = Rob(rob).sensors
                
                % Observe known landmarks
                [Rob(rob),Sen(sen),Lmk,Obs(sen,:),Frm(rob,Trj(rob).head),Fac] ...
                    = addKnownLmkFactors( ...
                    Rob(rob),   ...
                    Sen(sen),   ...
                    Raw(sen),   ...
                    Lmk,        ...
                    Obs(sen,:), ...
                    Frm(rob,Trj(rob).head), ...
                    Fac,        ...
                    Opt) ;
                
                % Initialize new landmarks
                ninits = Opt.init.nbrInits(1 + (currentFrame ~= Tim.firstFrame));
                for i = 1:ninits
                    
                    % Init new lmk
                    [Lmk,Obs(sen,:),Frm(rob,Trj(rob).head),Fac,lmk] = initNewLmk(...
                        Rob(rob),   ...
                        Sen(sen),   ...
                        Raw(sen),   ...
                        Lmk,        ...
                        Obs(sen,:), ...
                        Frm(rob,Trj(rob).head), ...
                        Fac,        ...
                        Opt) ;
                    
                    if isempty(lmk) % Did not find new lmks
                        break
                    end
                    
                end % for i = 1:ninits
                
            end % end process sensors
            
        end % end process robots
        
        % Solve graph
        [Rob,Sen,Lmk,Obs,Frm,Fac] = solveGraph(Rob,Sen,Lmk,Obs,Frm,Fac,Opt);
        
        % Reset odometer and sync robot with graph
        for rob = [Rob.rob]
            
            % Update robots with Frm info
            Rob(rob) = frm2rob(Rob(rob),Frm(rob,Trj(rob).head));
            
            % Reset motion robot
            factorRob(rob) = resetMotion(Rob(rob));
            
        end
        
    end
    
    % 3. SENSING + GP UPDATE
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (mod(num_control_frames-1, meas_frame_interval) == 0) || (do_first_meas)
        
        for rob = [Rob.rob]
            
            %keyboard
            
            [field_map, training_data] = ...
                take_measurement_at_point(Rob(rob), SimRob(rob), field_map, .....
                training_data, gt_data, testing_data, gp_params, map_params);
            
            metrics.times = [metrics.times; Map.t];
            metrics.points_meas = [metrics.points_meas; Rob(rob).state.x(1:3)'];
            metrics.points_meas_gt = [metrics.points_meas_gt; SimRob(rob).state.x(1:3)'];
            metrics.measurements = [metrics.measurements; training_data.Y_train(end)];
            metrics.P_traces = [metrics.P_traces; sum(field_map.cov)];
            metrics.rmses = [metrics.rmses; compute_rmse(field_map.mean, gt_data.Y_gt)];
            metrics.mlls = [metrics.mlls; compute_mll(field_map, gt_data.Y_gt)];
            r = Rob(rob).state.r(1:3);
            P = Map.P(r,r);
            metrics.Rob_Ps(:,:,size(metrics.times,1)) = P;
            
            disp(['Distance between real + estimated robot positions: ', ...
                num2str(pdist([Rob.state.x(1:3)'; SimRob.state.x(1:3)']))])
            disp(['Map RMSE = ', num2str(metrics.rmses(end))])
            
        end
        
        refresh_field_fig = 1;
        do_first_meas = 0;
        
    end
    
    % 4. PLANNING
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if isempty(points_control)
        
        % Greedy planner.
        %[~, max_ind] = max(field_map.cov);
        %[max_i, max_j, max_k] = ...
        %    ind2sub([dim_y, dim_x, dim_z], max_ind);
        %point_goal = ...
        %    grid_to_env_coordinates([max_j, max_i, max_k], map_params);
        %disp(['Next goal: ', num2str(point_goal)])
        
        % I. Grid search.
        path_points = search_lattice(Rob, lattice, field_map, ...
            SimLmk, Sen, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt, ...
            training_data, testing_data, map_params, planning_params, gp_params);
        
        % II. Trajectory optimization.
        if (strcmp(opt_params.opt_method, 'cmaes'))
            path_optimized = optimize_with_cmaes(path_points, field_map, ...
                Rob, Sen, SimLmk, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt, ...
                num_control_frames, training_data, testing_data, ...
                map_params, planning_params, opt_params, gp_params);
            
        else
            path_optimized = path_points;
        end
        
        disp('Next path: ')
        disp(path_optimized)
        disp(['Time: ', num2str(Map.t)])
        
        % Create polynomial path through the control points.
        trajectory = plan_path_waypoints(path_optimized, planning_params.max_vel, ...
            planning_params.max_acc);
        % Sample trajectory for motion simulation.
        [~, points_control, ~, ~] = ...
            sample_trajectory(trajectory, 1/planning_params.control_freq);
        
        metrics.path_travelled = [metrics.path_travelled; path_optimized];
        
        %keyboard
        
        %num_control_frames = 0;
        
    end
    
    
    % 5. VISUALIZATION
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if (currentFrame == Tim.firstFrame ...
            || currentFrame == Tim.lastFrame ...
            || mod(currentFrame,FigOpt.rendPeriod) == 0) ...
            && FigOpt.drawFigs
        
        
        % Figure of the Map:
        MapFig = drawMapFig(MapFig,  ...
            Rob, Sen, Lmk,  ...
            Trj, Frm, Fac, ...
            SimRob, SimSen, ...
            FigOpt);
        
        % Figure of the Field:
        if (refresh_field_fig)
            FldFig = drawFldFig(FldFig,  ...
                Rob, Lmk, ...
                SimRob, ...
                field_map.mean, field_map.cov, ...
                FigOpt);
            refresh_field_fig = 0;
        end
        
        if FigOpt.createVideo
            makeVideoFrame(MapFig, ...
                sprintf('map-%04d.png',currentFrame), ...
                FigOpt, ExpOpt);
        end
        
        % Figures for all sensors
        for sen = [Sen.sen]
            SenFig(sen) = drawSenFig(SenFig(sen), ...
                Sen(sen), Raw(sen), Obs(sen,:), ...
                FigOpt);
            
            if FigOpt.createVideo
                makeVideoFrame(SenFig(sen), ...
                    sprintf('sen%02d-%04d.png', sen, currentFrame),...
                    FigOpt, ExpOpt);
            end
            
        end
        
        % Do draw all objects
        drawnow;
        
    end
    
    % 5. DATA LOGGING
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end

%% VI. Post-processing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Enter post-processing code here



% ========== End of function - Start GPL license ==========


%   # START GPL LICENSE

%---------------------------------------------------------------------
%
%   This file is part of SLAMTB, a SLAM toolbox for Matlab.
%
%   SLAMTB is free software: you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation, either version 3 of the License, or
%   (at your option) any later version.
%
%   SLAMTB is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%
%   You should have received a copy of the GNU General Public License
%   along with SLAMTB.  If not, see <http://www.gnu.org/licenses/>.
%
%---------------------------------------------------------------------

%   SLAMTB is Copyright:
%   Copyright (c) 2008-2010, Joan Sola @ LAAS-CNRS,
%   Copyright (c) 2010-2013, Joan Sola,
%   Copyright (c) 2014-2015, Joan Sola @ IRI-UPC-CSIC,
%   SLAMTB is Copyright 2009
%   by Joan Sola, Teresa Vidal-Calleja, David Marquez and Jean Marie Codol
%   @ LAAS-CNRS.
%   See on top of this file for its particular copyright.

%   # END GPL LICENSE

