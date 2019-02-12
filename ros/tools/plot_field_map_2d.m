figure;

subplot(1,2,1)
scatter(testing_data.X_test(:,1), testing_data.X_test(:,2), ...
    200, field_map.mean, 'filled')
caxis([0 50])
axis([-1, 1, -1, 1])
colorbar
title('Mean')

subplot(1,2,2)
scatter(testing_data.X_test(:,1), testing_data.X_test(:,2), ...
    200, field_map.cov,'filled')
axis([-1, 1, -1, 1])
caxis([0 300])
colorbar
title('Covariance')
