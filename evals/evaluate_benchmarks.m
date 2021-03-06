%clear all; close all; clc;
warning('off','all')

% If data already exists, want to append to it for the trials it contains.
append_to_logger = 1;

% Number of trials to run
if (~append_to_logger)
    num_trials = 50;
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

evaluate_random = 1;
evaluate_rig = 1; subtree_iters = 30;

for i = 51:99
    
    if (~append_to_logger)
        t = i;
    else
        t = trials_names(i);
    end
    
    logger.(['trial', num2str(t)]).num = t;
    
    %% Random %%
    if (evaluate_random)
        rng(t, 'twister');
        [metrics] = slam_gp_random(map_params, planning_params, opt_params, gp_params, ...
            training_data, gt_data, testing_data);
        logger.(['trial', num2str(t)]).('random') = metrics;
        clear global
    end
    
    %% RIG-tree %%
    if (evaluate_rig)
   
        rng(t, 'twister');
        planning_params.obj_func = 'uncertainty';
        [metrics] = slam_gp_rig(map_params, planning_params, opt_params, gp_params, ...
            training_data, gt_data, testing_data, subtree_iters);
        logger.(['trial', num2str(t)]).('rig') = metrics;
        clear global
        
        rng(t, 'twister');
        planning_params.obj_func = 'renyi';
        [metrics] = slam_gp_rig(map_params, planning_params, opt_params, gp_params, ...
            training_data, gt_data, testing_data, subtree_iters);
        logger.(['trial', num2str(t)]).('rig_renyi') = metrics;
        clear global
    end
    
    save data.mat
    
    disp(['Completed Trial ', num2str(t)])
    
end
