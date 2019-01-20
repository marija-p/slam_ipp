trials = fieldnames(logger);
trials = regexp(trials,'\d*','Match');
trials = [trials{:}];
trials_names = [];
for i = 1:length(trials)
    trials_names = ...
        [trials_names; str2num(cell2mat(trials(i)))];
end

methods = {'no_UI', 'no_UI_adaptive', 'UI_N_gauss_9', 'UI_N_gauss_9_adaptive'};

interesting_ind = find(gt_data.Y_gt >= planning_params.lower_thres);
uninteresting_ind = find(gt_data.Y_gt < planning_params.lower_thres);

for t = 1:length(trials)
    
    
    for j = 1:length(methods)
        
        
        measurement_points = ...
            logger.((['trial', num2str(t)])).(methods{j}).points_meas;
        measurements = ...
            logger.((['trial', num2str(t)])).(methods{j}).measurements;
        Rob_Ps = ...
            logger.((['trial', num2str(t)])).(methods{j}).Rob_Ps;
        logger.(['trial', num2str(t)]).(methods{j}).P_traces_interesting = [];
        logger.(['trial', num2str(t)]).(methods{j}).P_traces_uninteresting = [];
        logger.(['trial', num2str(t)]).(methods{j}).rmses_interesting = [];
        logger.(['trial', num2str(t)]).(methods{j}).rmses_uninteresting = [];
        
        gp_params = logger.(['trial', num2str(t)]).(methods{j}).gp_params;
        
        for k = 1:size(measurement_points, 1)
            
            training_data.X_train = measurement_points(1:k,:);
            training_data.Y_train = measurements(1:k);
            P = Rob_Ps(:,:,k);
            
            % Do GP regression and update the field map.
            if (gp_params.use_modified_kernel)
                cov_func = {@covUI, gp_params.cov_func, gp_params.N_gauss, P};
            else
                cov_func = gp_params.cov_func;
            end
            [ymu, ys, fmu, fs, ~ , post] = gp(gp_params.hyp_trained, ...
                gp_params.inf_func, gp_params.mean_func, cov_func, gp_params.lik_func, ...
                training_data.X_train, training_data.Y_train, testing_data.X_test);
            field_map.mean = ymu;
            field_map.cov = ys;
            
            P_trace_interesting = sum(field_map.cov(interesting_ind));
            P_trace_uninteresting = sum(field_map.cov(uninteresting_ind));
            rmse_interesting = ...
                compute_rmse(gt_data.Y_gt(interesting_ind), field_map.mean(interesting_ind));
            rmse_uninteresting = ...
                compute_rmse(gt_data.Y_gt(uninteresting_ind), field_map.mean(uninteresting_ind));
            
            logger.((['trial', num2str(t)])).(methods{j}).P_traces_interesting = ...
                [logger.(['trial', num2str(t)]).(methods{j}).P_traces_interesting; ...
                P_trace_interesting];
            logger.((['trial', num2str(t)])).(methods{j}).P_traces_uninteresting = ...
                [logger.(['trial', num2str(t)]).(methods{j}).P_traces_uninteresting; ...
                P_trace_uninteresting];
            logger.((['trial', num2str(t)])).(methods{j}).rmses_interesting = ...
                [logger.(['trial', num2str(t)]).(methods{j}).rmses_interesting; ...
                rmse_interesting];
            logger.((['trial', num2str(t)])).(methods{j}).rmses_uninteresting = ...
                [logger.(['trial', num2str(t)]).(methods{j}).rmses_uninteresting; ...
                rmse_uninteresting];
            
        end
        
    end
    
    disp(['Completed Trial ', num2str(t), '!'])
    
end