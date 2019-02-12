figure;

subplot(1,2,1)
%scatter(testing_data.X_test(:,1), testing_data.X_test(:,2), ...
%    200, field_map.mean, 'filled')
imagesc(reshape(field_map.mean, map_params.dim_y, map_params.dim_x));
caxis([0 50])
%axis([-1, 1, -1, 1])
colorbar
set(gca, 'YDir', 'Normal')
axis off
title('Mean')

subplot(1,2,2)
imagesc(reshape(field_map.cov, map_params.dim_y, map_params.dim_x));
%axis([-1, 1, -1, 1])
caxis([0 300])
colorbar
set(gca, 'YDir', 'Normal')
axis off
title('Covariance')

set(gcf, 'Position', [-468   499   938   376])