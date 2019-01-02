figure;

points_meas = logger.trial6.UI_N_gauss_5.points_meas;
measurements = logger.trial6.UI_N_gauss_5.measurements;

training_data.X_train = points_meas;
training_data.Y_train = measurements;

[ymu, ys, ~, ~, ~ , ~] = gp(gp_params.hyp_trained, ...
    gp_params.inf_func, gp_params.mean_func, gp_params.cov_func1, gp_params.lik_func, ...
    training_data.X_train, training_data.Y_train, testing_data.X_test);
field_map.mean = ymu;
field_map.cov = ys;

subplot(1,2,1)
scatter3(testing_data.X_test(:,1), testing_data.X_test(:,2), ...
    testing_data.X_test(:,3), 60, field_map.mean,'filled')
caxis([0 50])
colorbar
title('Mean')

subplot(1,2,2)
scatter3(testing_data.X_test(:,1), testing_data.X_test(:,2), ...
    testing_data.X_test(:,3), 60, field_map.cov,'filled')
caxis([0 300])
colorbar
title('Covariance')

disp(['Map RMSE = ', num2str(compute_rmse(ymu, gt_data.Y_gt))])
