function field_fig = update_field_fig(field_fig, field_map, path_points, points_meas, map_params)

field_fig.mean.CData = reshape(field_map.mean, map_params.dim_y, map_params.dim_x);
field_fig.cov.CData = reshape(field_map.cov, map_params.dim_y, map_params.dim_x);

field_fig.points_meas.XData = points_meas(:,1);
field_fig.points_meas.YData = points_meas(:,2);

field_fig.path.XData = path_points(:,1);
field_fig.path.YData = path_points(:,2);

end