figure;

subplot(1,2,1)
title('Mean')
scatter(testing_data.X_test(:,1), testing_data.X_test(:,2), ...
    60, field_map.mean, 'filled')
caxis([0 50])
axis([-1, 1, -1, 1])
colorbar

subplot(1,2,2)
title('Covariance')
scatter(testing_data.X_test(:,1), testing_data.X_test(:,2), ...
    60, field_map.cov,'filled')
axis([-1, 1, -1, 1])
caxis([0 300])
colorbar
