function field_fig = create_field_fig(points_meas, map_params)

field_fig.fig = figure;

subplot(1,2,1)
%scatter(testing_data.X_test(:,1), testing_data.X_test(:,2), ...
%    200, field_map.mean, 'filled')
field_fig.mean = imagesc(zeros(map_params.dim_y, map_params.dim_x));
caxis([0 50])
%axis([-1, 1, -1, 1])
colorbar
set(gca, 'YDir', 'Normal')
axis off
title('Mean')

subplot(1,2,2)
field_fig.cov = imagesc(zeros(map_params.dim_y, map_params.dim_x));
%axis([-1, 1, -1, 1])
caxis([0 300])
colorbar
set(gca, 'YDir', 'Normal')
axis off
title('Covariance')

field_fig.points_meas = plot(points_meas(:,1), points_meas(:,2), 'ok', 'MarkerSize', 10);
field_fig.path = plot(points_meas(:,1), points_meas(:,2), '--k', 'LineWidth', 1);

set(gcf, 'Position', [-468   499   938   376])

end

