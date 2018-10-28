function field_map = predict_map_update(pos, field_map, ...
    training_data, testing_data, map_params, gp_params)
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
%r = Rob.state.r(1:3);
%P = Map.P(r,r);
% Take maximum likelihood measurement based on current field map state.
training_data.Y_train = [training_data.Y_train; ...
    interp3(reshape(testing_data.X_test(:,1),dim_y,dim_x,dim_z), ...
    reshape(testing_data.X_test(:,2),dim_y,dim_x,dim_z), ...
    reshape(testing_data.X_test(:,3),dim_y,dim_x,dim_z), ...
    reshape(field_map.mean,dim_y,dim_x,dim_z), ...
    pos(1), pos(2), pos(3))];

% Do GP regression and update the field map.
%cov_func_UI = {@covUI, gp_params.cov_func, gp_params.N_gauss, P};
[ymu, ys, ~, ~, ~ , ~] = gp(gp_params.hyp_trained, ...
    gp_params.inf_func, gp_params.mean_func, gp_params.cov_func, gp_params.lik_func, ...
    training_data.X_train, training_data.Y_train, testing_data.X_test);
field_map.mean = ymu;
field_map.cov = ys;

end

