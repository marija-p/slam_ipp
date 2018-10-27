function field_map = predict_map_update(pos, field_map, ...
    training_data, testing_data, gp_params)
% Predicts grid map update at an unvisited UAV position using GP
% regression.
% --
% Inputs:
% pos: current [x,y,z] UAV position [m] - env. coordinates
% grid_map: current grid map (mean + covariance)
% ---
% Outputs:
% grid map
% ---
% M Popovic 2018
%

global Map
dim_x = map_params.dim_x;
dim_y = map_params.dim_y;
dim_z = map_params.dim_z;

% Update training data.
training_data.X_train = [training_data.X_train; pos];
r = Rob.state.r(1:3);
P = Map.P(r,r);
% Take maximum likelihood measurement based on current field map state.
training_data.Y_train = [training_data.Y_train; ...
    interp3(reshape(gt_data.X_gt(:,1),dim_y,dim_x,dim_z), ...
    reshape(gt_data.X_gt(:,2),dim_y,dim_x,dim_z), ...
    reshape(gt_data.X_gt(:,3),dim_y,dim_x,dim_z), ...
    reshape(gt_data.Y_gt,dim_y,dim_x,dim_z), ...
    SimRob.state.x(1), SimRob.state.x(2), SimRob.state.x(3))];

outputArg1 = inputArg1;
outputArg2 = inputArg2;
end

