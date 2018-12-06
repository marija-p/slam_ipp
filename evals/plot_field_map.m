figure;

subplot(1,2,1)
scatter3(testing_data.X_test(:,1), testing_data.X_test(:,2), ...
    testing_data.X_test(:,3), 60, field_map.mean,'filled')
caxis([0 50])
colorbar

subplot(1,2,2)
scatter3(testing_data.X_test(:,1), testing_data.X_test(:,2), ...
    testing_data.X_test(:,3), 60, field_map.cov,'filled')
caxis([0 2*10^4])
colorbar
