classdef RIGTree
    
    properties
        
        % Default start
        start = [-2 -2 1];
        
        % Default state space [xmin xmax ymin ymax zmin zmax]
        space = [-2.5 2.5 -2.5 2.5 1 5];
        
        % RRT expansion step size
        q_delta = 4;
        
        % Tree structure
        rigtree = struct('vertices', [],'numvertices',[], 'edges',[], ...
            'vertices_closed', []);
    end
    
    methods
        
        % Sets the start position.
        function setStart(self, start)
            self.start = start;
        end
        
        % Sets the state space boundaries.
        function setSpace(self, xmin, xmax, ymin, ymax, zmin, zmax)
            
            if (xmin < xmax && ymin < ymax && zmin < zmax)
                self.space = [xmin xmax ymin ymax zmin zmax];
            else
                error('Invalid minimum and maximum.');
            end
            
        end
        
        % Samples uniformly in state-space.
        function q_rand = sampleLocation(self)
            
            x = self.space(1) + (self.space(2)-self.space(1))*rand;
            y = self.space(3) + (self.space(4)-self.space(3))*rand;
            z = self.space(5) + (self.space(6)-self.space(5))*rand;
            q_rand = [x y z];
            
        end
        
       % Adds an edge from q_nearest to q_new to the tree.
       function edge_to_add = addEdge(self, q_nearest_idx, q_new)
           
           q_nearest = self.rigtree.vertices(q_nearest_idx);
           rigtree_edge = q_new.location - q_nearest.location;
           edge_start_idx = q_nearest_idx;
           edge_goal_idx = self.rigtree.numvertices;
           edge_to_add = table(rigtree_edge, edge_start_idx, edge_goal_idx);
           
        end
        
        % Finds the nearest neighbor. Naive search.
        function [q_nearest, q_nearest_idx] = findNearestVertex(self, x)
            
            % Get locations of open vertices.
            vertices_open_idx = getOpenVertexIndices(self);
            vertices_open = self.rigtree.vertices(vertices_open_idx);
            vertices_open_locations = getVertexLocations(self, vertices_open);
            
            % Find open vertex closest to target location.
            distances = pdist2(vertices_open_locations, x); 
            [~, d_min_idx] = min(distances);
            q_nearest_idx = vertices_open_idx(d_min_idx);
            q_nearest = self.rigtree.vertices(q_nearest_idx);
            
        end
        
        % Steps a fixed distance from a tree node in a random direction.
        function [x_feasible] = stepToLocation(self, x_near, x_rand)
            
            distance = pdist([x_near; x_rand]);
            
            if distance < self.q_delta
                x_feasible = x_rand;
            else
                x_feasible = x_near + (self.q_delta/distance) * (x_rand - x_near);
            end
            
        end
        
        % Finds near points to be extended.
        function [neighbors_idx] = findNeighborIndices(self, x_feasible)
            
            % Compute RRT* radius.
            r = 5*(log(self.rigtree.numvertices+1)/self.rigtree.numvertices+1)^1/3;
            %r = 100;
            
            % Get locations of open vertices.
            vertices_open_idx = getOpenVertexIndices(self);
            vertices_open = self.rigtree.vertices(vertices_open_idx);
            vertices_open_locations = getVertexLocations(self, vertices_open);
            
            % Find all near points given the radius.
            neighbors_idx = rangesearch(vertices_open_locations, x_feasible, r);
            neighbors_idx = neighbors_idx{:};
            
        end
        
        % Decides if a target vertex should be pruned, depending on if a
        % better co-located one exists in the tree
        % (conservative pruning function).
        function [prune] = pruneVertex(self, q)
            
          vertices_locations = getVertexLocations(self, self.rigtree.vertices);
          [~, idx] = find(ismember(q.location, vertices_locations, 'rows'));
          
          prune = 0;
          
          if(~isempty(idx))
              keyboard
          end
          
          for i = 1:size(idx)
              
              if (q.gain <= self.rigtree.vertices(idx(i)).gain && ...
                      q.cost >= self.rigtree.vertices(idx(i)).cost)
                  prune = 1;
                  return;
              end
              
          end
          
        end
        
        % Traces path to starting location from an indexed vertex.
        function [path] = tracePath(self, q_goal_idx)
         
            % Find first parent from list od edges.
            row_idx = ...
                (self.rigtree.edges.edge_goal_idx == q_goal_idx);
            q_start_idx = self.rigtree.edges.edge_start_idx(row_idx);
            path = [self.rigtree.vertices(q_start_idx), self.rigtree.vertices(q_goal_idx)];
            
            % Loop backwards through parents.
            while (~isempty(q_start_idx))
                
                q_goal_idx = q_start_idx;
                row_idx = ...
                    (self.rigtree.edges.edge_goal_idx == q_goal_idx);
                q_start_idx = self.rigtree.edges.edge_start_idx(row_idx);
                path = [self.rigtree.vertices(q_start_idx), path];
                
            end
            
        end
        
        % Gets a list of open vertices in the tree.
        function [idx] = getOpenVertexIndices(self)
            
            vertices_locations = getVertexLocations(self, self.rigtree.vertices);
            
            % There are closed vertices
            if (~isempty(self.rigtree.vertices_closed))
                vertices_closed_locations = ...
                    getVertexLocations(self, self.rigtree.vertices_closed);
                [~, idx] = ...
                    setdiff(vertices_locations, vertices_closed_locations, 'rows');
            % There are no closed vertices
            else
                idx = 1:length(self.rigtree.vertices);
            end
            
        end
        
        % Gets the (x,y,z) locations of a list of N vertices as a Nx3
        % matrix.
        function [vertices_locations] = getVertexLocations(~, vertices)
            
            vertices_locations = cell2mat({vertices(:).location});
            vertices_locations = reshape(vertices_locations, 3, [])';
        
        end
        
        % Re-initializes the tree to default values.
        function [self] = resetTree(self)
            
            self.rigtree.vertices = [];
            self.rigtree.numvertices = [];
            self.rigtree.edges = [];
            self.rigtree.vertices_closed = [];
            
        end
        
    end

end

