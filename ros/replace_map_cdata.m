%if (gp_params.use_modified_kernel)
%    gp_params.cov_func = {@covUI, gp_params.cov_func, gp_params.N_gauss, Rob_P(1:2,1:2)};
%end

ind = 15;

disp(['Time = ', num2str(metrics.times(ind))])

training_data.X_train = metrics.points_meas(1:ind,:);
training_data.Y_train = metrics.measurements(1:ind);
[ymu, ys, ~, ~, ~ , ~] = gp(gp_params.hyp_trained, ...
    gp_params.inf_func, gp_params.mean_func, gp_params.cov_func, gp_params.lik_func, ...
    training_data.X_train, training_data.Y_train, testing_data.X_test);

set(gco, 'CData', reshape(ymu,7,7));