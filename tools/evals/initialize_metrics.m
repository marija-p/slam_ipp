function metrics = initialize_metrics()

metrics.path_travelled = [];

metrics.times = [];
metrics.points_meas = [];
metrics.P_traces = [];
metrics.rmses = [];
metrics.mlls = [];
metrics.Rob_Ps = zeros(3,3,200);

end

