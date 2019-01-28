%clear all; close all; clc;
warning('off','all')

% If data already exists, want to append to it for the trials it contains.
append_to_logger = 1;

% Number of trials to run
if (~append_to_logger)
    num_trials = 100;
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

for i = 1:num_trials
    
    if (~append_to_logger)
        t = i;
    else
        t = trials_names(i);
    end
    
    logger.(['trial', num2str(t)]).num = t;
 
    %% Random %%
    rng(t, 'twister');
    [metrics] = slam_gp_random(map_params, planning_params, opt_params, gp_params, ...
        training_data, gt_data, testing_data);
    logger.(['trial', num2str(t)]).('random') = metrics;
    clear global
    
    save data.mat
    
    disp(['Completed Trial ', num2str(t)])
    
end