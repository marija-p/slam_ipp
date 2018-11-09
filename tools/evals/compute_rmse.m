function rmse = compute_rmse(field_map_mean, ground_truth_map)
% Computes the RMSE of a map wrt. ground truth.

se = sum((field_map_mean - ground_truth_map).^2);
mse = se / numel(ground_truth_map);
rmse = sqrt(mse);

end