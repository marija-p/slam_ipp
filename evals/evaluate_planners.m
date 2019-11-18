%clear all; close all; clc;
warning('off','all')

% If data already exists, want to append to it for the trials it contains.
append_to_logger = 0;

% Number of trials to run
if (~append_to_logger)
    num_trials = 60;
else
    trials = fieldnames(logger);
    trials = regexp(trials,'\d*','Match');
    trials = [trials{:}];
    trials_names = [];
    for i = 1:length(trials)
        trials_names = ...
            [trials_names; str2num(cell2mat(trials(i)))];
    end
    num_trials = length(trials);
end

% UAV workspace dimensions [m]
dim_x_env = 5;
dim_y_env = 5;
dim_z_env = 4;

[map_params, planning_params, opt_params, gp_params, ...
    training_data, gt_data, testing_data] = ...
    load_params_and_data(dim_x_env, dim_y_env, dim_z_env);

debug_file = fopen('map_rmses.txt', 'w');

for i = 1:num_trials
    
    if (~append_to_logger)
        t = i;
    else
        t = trials_names(i);
    end
    
    logger.(['trial', num2str(t)]).num = t;
    
    %% Uncertainty %%
    planning_params.obj_func = 'uncertainty';
    %     rng(t, 'twister');
    %     gp_params.use_modified_kernel = 0;
    %     gp_params.use_modified_kernel_prediction = 0;
    %     [metrics] = slam_gp(map_params, planning_params, opt_params, gp_params, ...
    %         training_data, gt_data, testing_data);
    %     logger.(['trial', num2str(t)]).('uncertainty_no_UI') = metrics;
    %     clear global
    %     fprintf(debug_file, 'uncertainty_no_UI\n');
    %     fprintf(debug_file, '%f %f\n', [metrics.times'; metrics.rmses']);
    % %
    rng(t, 'twister');
    gp_params.use_modified_kernel = 1;
    gp_params.use_modified_kernel_prediction = 1;
    gp_params.N_gauss = 5;
    [metrics] = slam_gp(map_params, planning_params, opt_params, gp_params, ...
         training_data, gt_data, testing_data);
     logger.(['trial', num2str(t)]).('uncertainty_UI_N_gauss_5') = metrics;
     clear global
     fprintf(debug_file, 'uncertainty_UI_N_gauss_5\n');
     fprintf(debug_file, '%f %f\n', [metrics.times'; metrics.rmses']);
     
     %% Renyi %%
     planning_params.obj_func = 'renyi';
 
     %rng(t, 'twister');
     %gp_params.use_modified_kernel = 0;
     %gp_params.use_modified_kernel_prediction = 0;
     %[metrics] = slam_gp(map_params, planning_params, opt_params, gp_params, ...
     %    training_data, gt_data, testing_data);
     %logger.(['trial', num2str(t)]).('renyi_no_UI') = metrics;
     %clear global
     %fprintf(debug_file, 'renyi_no_UI\n');
     %fprintf(debug_file, '%f %f\n', [metrics.times'; metrics.rmses']);
     
     rng(t, 'twister');
     gp_params.use_modified_kernel = 1;
     gp_params.use_modified_kernel_prediction = 1;
     gp_params.N_gauss = 5;
     [metrics] = slam_gp(map_params, planning_params, opt_params, gp_params, ...
         training_data, gt_data, testing_data);
     logger.(['trial', num2str(t)]).('renyi_UI_N_gauss_5') = metrics;
     clear global
     fprintf(debug_file, 'renyi_UI_N_gauss_5\n');
     fprintf(debug_file, '%f %f\n', [metrics.times'; metrics.rmses']);

     %% Heuristic %%
     %planning_params.obj_func = 'heuristic';
     
     %rng(t, 'twister');
     %gp_params.use_modified_kernel = 1;
     %gp_params.use_modified_kernel_prediction = 1;
     %gp_params.N_gauss = 5;
     %[metrics] = slam_gp(map_params, planning_params, opt_params, gp_params, ...
     %    training_data, gt_data, testing_data);
     %logger.(['trial', num2str(t)]).('heuristic_UI_N_gauss_5') = metrics;
     %clear global
     %fprintf(debug_file, 'heuristic_UI_N_gauss_5\n');
     %fprintf(debug_file, '%f %f\n', [metrics.times'; metrics.rmses']);
     
     save data_env10.mat
     
     disp(['Completed Trial ', num2str(t)])
     
end