function [field_map, training_data] = ...
    take_measurement_at_point_ros(pos, Rob_P, measurement, field_map, ...
    training_data, testing_data, gp_params)
% Take measurement at estimated robot pose and update GP field (2D).

% Update training data with real measurement.
training_data.X_train = [training_data.X_train; pos];
training_data.P_train(:,:,size(training_data.X_train,1)) = Rob_P;
training_data.Y_train = [training_data.Y_train; measurement];

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
