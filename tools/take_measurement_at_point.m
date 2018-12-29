function [field_map, training_data] = ...
    take_measurement_at_point(pos, pos_gt, Rob_P, field_map, ...
    training_data, gt_data, testing_data, gp_params, map_params)
% Take measurement at estimated robot pose and update GP field.

dim_x = map_params.dim_x;
dim_y = map_params.dim_y;
dim_z = map_params.dim_z;

% Update training data.
training_data.X_train = [training_data.X_train; pos];
training_data.X_train_gt = [training_data.X_train_gt; pos_gt];
training_data.P_train(:,:,size(training_data.X_train,1)) = Rob_P;
% Take measurement value from real robot state.
training_data.Y_train = [training_data.Y_train; ...
    interp3(reshape(gt_data.X_gt(:,1),dim_y,dim_x,dim_z), ...
    reshape(gt_data.X_gt(:,2),dim_y,dim_x,dim_z), ...
    reshape(gt_data.X_gt(:,3),dim_y,dim_x,dim_z), ...
    reshape(gt_data.Y_gt,dim_y,dim_x,dim_z), ...
    pos_gt(1), pos_gt(2), pos_gt(3), 'spline')];

% Do GP regression and update the field map.
if (gp_params.use_modified_kernel)
    gp_params.cov_func = {@covUI, gp_params.cov_func, gp_params.N_gauss, Rob_P};
end
[ymu, ys, ~, ~, ~ , ~] = gp(gp_params.hyp_trained, ...
    gp_params.inf_func, gp_params.mean_func, gp_params.cov_func, gp_params.lik_func, ...
    training_data.X_train, training_data.Y_train, testing_data.X_test);
field_map.mean = ymu;
field_map.cov = ys;

end
