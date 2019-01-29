classdef Vertex
    
    properties
        
        % Coordinates in configuration space (x,y,z)
        location;
        % Informative objective at this point
        gain;
        cost;
        objective;
        
    end
    
    methods
        
        % Evaluates informative objective at a vertex.
        function self = evaluateObjective(self, q_near_idx, tree, field_map, ...
                Rob, Sen, SimLmk, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt, ...
                num_control_frames, current_frame, training_data, testing_data, ...
                map_params, planning_params, gp_params)
            
            % Add current vertex to the tree.
            tree.rigtree.vertices = ...
                [tree.rigtree.vertices; self];
            tree.rigtree.numvertices = ...
                tree.rigtree.numvertices + 1;
            tree.rigtree.edges = [tree.rigtree.edges; ...
                tree.addEdge(q_near_idx, self)];
            
            % Find path to current vertex in tree.
            control_vertices = tree.tracePath(tree.rigtree.numvertices);
            control_points = tree.getVertexLocations(control_vertices);
            trajectory = ...
                plan_path_waypoints(control_points, ...
                planning_params.max_vel, planning_params.max_acc);
            
            % Sample trajectory to find locations to take measurements at.
            [~, control_points, ~, ~] = ...
                sample_trajectory(trajectory, 1/planning_params.control_freq);
            
            if size(control_points,1) < 2
                self.objective = Inf;
                return;
            end
            
            % Control/measurement simulation.
            self.objective = compute_objective(control_points, field_map, ...
                Rob, Sen, SimLmk, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt, ...
                num_control_frames, current_frame, training_data, testing_data, ...
                map_params, planning_params, gp_params);
            
        end
        
        function r = eq(a, b)
            
            if (isempty(b))
                r = false;
                return;
            end
            
            if (a.location == b.location)
                r = true;
            else
                r = false;
            end
            
        end
        
    end
    
end

