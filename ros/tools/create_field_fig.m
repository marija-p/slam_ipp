function field_fig = create_field_fig(points_meas, map_params)

pos_x = map_params.pos_x;
pos_y = map_params.pos_y;
res_x = map_params.res_x;
res_y = map_params.res_y;
dim_x = map_params.dim_x;
dim_y = map_params.dim_y;

field_fig.fig = figure;

subplot(1,2,1)
hold on
%scatter(testing_data.X_test(:,1), testing_data.X_test(:,2), ...
%    200, field_map.mean, 'filled')
field_fig.mean = imagesc([pos_x pos_x+dim_x*res_x], ...
    [pos_y pos_y+dim_y*res_y], zeros(map_params.dim_y, map_params.dim_x));
axis([pos_x pos_x+dim_x*res_x pos_y pos_y+dim_y*res_y])
caxis([22 28])
%axis([-1, 1, -1, 1])
colorbar
set(gca, 'YDir', 'Normal')
axis off
title('Mean')

field_fig.points_meas = plot(points_meas(:,1), points_meas(:,2), 'or', 'MarkerSize', 15);
field_fig.path = plot(points_meas(:,1), points_meas(:,2), '--k', 'LineWidth', 2);

subplot(1,2,2)
field_fig.cov = imagesc([pos_x pos_x+dim_x*res_x], ...
    [pos_y pos_y+dim_y*res_y], zeros(map_params.dim_y, map_params.dim_x));
axis([pos_x pos_x+dim_x*res_x pos_y pos_y+dim_y*res_y])
caxis([0 2])
colorbar
set(gca, 'YDir', 'Normal')
axis off
title('Covariance')

set(gcf, 'Position', [-468   499   938   376])

end

