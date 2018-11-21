function metrics = ...
    initialize_metrics(map_params, planning_params, opt_params, gp_params)

metrics.path_travelled = [];

metrics.times = [];
metrics.points_meas = [];
metrics.points_meas_gt = [];
metrics.measurements = [];
metrics.P_traces = [];
metrics.rmses = [];
metrics.mlls = [];
metrics.Rob_Ps = zeros(3,3,200);

metrics.map_params = map_params;
metrics.planning_params = planning_params;
metrics.opt_params = opt_params;
metrics.gp_params = gp_params;

end

