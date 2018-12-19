file_path = '~\PhD\Submissions\asldoc-2017-iros-popovic\images\';

rescale_factor = 1;
%rescale_factor = 0.75;
text_size = 10.5;

do_plot = 1;
do_print = 0;
show_legend = 1;

paper_pos = [0, 0, 6, 4];

percentile = 0.99;

trials = fieldnames(logger);
methods = fieldnames(logger.trial1);

% Last trial is incomplete.
if (length(methods) ~= length(fieldnames(logger.(trials{end}))))
    trials = trials(1:end-1);
end
disp(['Number of trials = ', num2str(length(trials))])

time_vector = 0:0.1:200;

P_traces = zeros(length(methods)-1,length(time_vector));
rmses = zeros(length(methods)-1,length(time_vector));
mlls = zeros(length(methods)-1,length(time_vector));
Rob_Ps_Aopt = zeros(length(methods)-1,length(time_vector));
Rob_Ps_Dopt = zeros(length(methods)-1,length(time_vector));
pose_errors = zeros(length(methods)-1,length(time_vector));

for i = 1:length(trials)
    
    for j = 2:length(methods)
        
        try
            time = logger.(trials{i}).(methods{j}).times;
        catch
            disp(['Cant find ', trials{i}, ' ' methods{j}])
            break;
        end
        
        P_trace = logger.(trials{i}).(methods{j}).P_traces;
        rmse = logger.(trials{i}).(methods{j}).rmses;
        mll = logger.(trials{i}).(methods{j}).mlls;
        
        Rob_P_Aopt = [];
        Rob_P_Dopt = [];
        
        for k = 1:size(metrics.times,1)
            Rob_P_Aopt = [Rob_P_Aopt; trace(logger.(trials{i}).(methods{j}).Rob_Ps(:,:,k))];
            Rob_P_Dopt = [Rob_P_Dopt; det(logger.(trials{i}).(methods{j}).Rob_Ps(:,:,k))];
        end

        pose_error = sqrt(sum((logger.(trials{i}).(methods{j}).points_meas - ...
            logger.(trials{i}).(methods{j}).points_meas_gt).^2, 2));
        
        ts = timeseries(P_trace, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        P_traces(j-1,:,i) = ts_resampled.data';
        
        ts = timeseries(rmse, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        rmses(j-1,:,i) = ts_resampled.data';
        
        ts = timeseries(mll, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        mlls(j-1,:,i) = ts_resampled.data';
        
        ts = timeseries(Rob_P_Aopt, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        Rob_Ps_Aopt(j-1,:,i) = ts_resampled.data';
        
        ts = timeseries(Rob_P_Dopt, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        Rob_Ps_Dopt(j-1,:,i) = ts_resampled.data';

        ts = timeseries(pose_error, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        pose_errors(j-1,:,i) = ts_resampled.data';

    end
    
end

% Find means and medians.
mean_P_traces = sum(P_traces,3)./length(trials);
mean_rmses = sum(rmses,3)./length(trials);
mean_mlls = sum(mlls,3)./length(trials);
mean_Rob_Ps_Aopt = sum(Rob_Ps_Aopt,3)./length(trials);
mean_Rob_Ps_Dopt = sum(Rob_Ps_Dopt,3)./length(trials);
mean_pose_errors = sum(pose_errors,3)./length(trials);
median_P_traces = median(P_traces,3);
median_rmses = median(rmses,3);
median_mlls = median(mlls,3);
median_pose_errors = median(pose_errors,3);

% Find confidence intervals
% http://ch.mathworks.com/matlabcentral/answers/159417-how-to-calculate-the-confidence-interva
SEM_P_traces = [];
SEM_rmses = [];
SEM_mlls = [];
SEM_Rob_Ps_Aopt = [];
SEM_Rob_Ps_Dopt = [];
SEM_pose_errors = [];

for j = 2:length(methods)
    
    SEM_P_traces(j-1,:) = std(squeeze(P_traces(j-1,:,:))', 'omitnan')/...
        sqrt(length(trials));
    SEM_rmses(j-1,:) = (std(squeeze(rmses(j-1,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    SEM_mlls(j-1,:) = (std(squeeze(mlls(j-1,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    SEM_Rob_Ps_Aopt(j-1,:) = (std(squeeze(Rob_Ps_Aopt(j-1,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    SEM_Rob_Ps_Dopt(j-1,:) = (std(squeeze(Rob_Ps_Dopt(j-1,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    SEM_pose_errors(j-1,:) = (std(squeeze(pose_errors(j-1,:,:))', 'omitnan')/...
        sqrt(length(trials)));    
end

% Symmetric
ts = tinv(percentile, length(trials));

colours = [0    0.4470    0.7410;
    0.8500    0.3250    0.0980;
    0.9290    0.6940    0.1250;
    0.4940    0.1840    0.5560;
    0.4660    0.6740    0.1880];
%0.6350    0.0780    0.1840;
%0.3010    0.7450    0.9330;
%0.1379    0.1379    0.0345];
transparency = 0.3;


%% PLOTTING %%

if (do_plot)
    
    figure;
    %% GP field covariance trace %%
    subplot(2,3,1)
    hold on
    h = zeros(length(methods)-1,1);
    if length(methods)-1 == 3
        boundedline(time_vector, mean_P_traces(1,:), SEM_P_traces(1,:)*ts, ...
            time_vector, mean_P_traces(2,:), SEM_P_traces(2,:)*ts, ...
            time_vector, mean_P_traces(3,:), SEM_P_traces(3,:)*ts, ...
            'alpha', 'cmap', colours, 'transparency', transparency);
    elseif length(methods)-1 == 2
        boundedline(time_vector, mean_P_traces(1,:), SEM_P_traces(1,:)*ts, ...
            time_vector, mean_P_traces(2,:), SEM_P_traces(2,:)*ts, ...
            'alpha', 'cmap', colours, 'transparency', transparency);
    end
    
    for i = 1:length(methods)-1
        P_trace = mean_P_traces(i,:);
        h(i) = plot(time_vector, P_trace, 'LineWidth', 1, 'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('Tr(P)');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YScale'      , 'log'     , ...
        'YGrid'       , 'on'      , ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size, ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    
    axis([0 time_vector(end) 0 6*10^7])
    rescale_axes(rescale_factor);
    %   pbaspect(gca, [1 2 1])
    hold off
    
    %% RMSE %%
    subplot(2,3,2)
    hold on
    if length(methods)-1 == 3
        boundedline(time_vector, mean_rmses(1,:), SEM_rmses(1,:)*ts, ...
            time_vector, mean_rmses(2,:), SEM_rmses(2,:)*ts, ...
            time_vector, mean_rmses(3,:), SEM_rmses(3,:)*ts, ...
            'alpha', 'cmap', colours, 'transparency', transparency);
    elseif length(methods)-1 == 2
        boundedline(time_vector, mean_rmses(1,:), SEM_rmses(1,:)*ts, ...
            time_vector, mean_rmses(2,:), SEM_rmses(2,:)*ts, ...
            'alpha', 'cmap', colours, 'transparency', transparency);
    end
    
    for i = 1:length(methods)-1
        rmse = mean_rmses(i,:);
        h(i) = plot(time_vector, rmse, 'LineWidth', 1, 'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('RMSE');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YTick'       , 0:1:8, ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size, ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    rescale_axes(rescale_factor);
    axis([0 time_vector(end) 1 8.5])
    %    pbaspect(gca, [1 2 1])
    hold off
    
    %% MLL %%
    subplot(2,3,3)
    hold on
    if length(methods)-1 == 3
        boundedline(time_vector, mean_mlls(1,:), SEM_mlls(1,:)*ts, ...
            time_vector, mean_mlls(2,:), SEM_mlls(2,:)*ts, ...
            time_vector, mean_mlls(3,:), SEM_mlls(3,:)*ts, ...
            'alpha', 'cmap', colours, 'transparency', transparency);
    elseif length(methods)-1 == 2
        boundedline(time_vector, mean_mlls(1,:), SEM_mlls(1,:)*ts, ...
            time_vector, mean_mlls(2,:), SEM_mlls(2,:)*ts, ...
            'alpha', 'cmap', colours, 'transparency', transparency);
    end
    
    for i = 1:length(methods)-1
        mll = mean_mlls(i,:);
        h(i) = plot(time_vector, mll, 'LineWidth', 1, 'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('MLL');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YTick'       , 0:2:10, ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size, ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    rescale_axes(rescale_factor);
    axis([0 time_vector(end) 0 10])
    %    pbaspect(gca, [1 2 1])
    hold off
    
    %% Robot covariance trace (A-opt) %%
    subplot(2,3,4)
    hold on
    if length(methods)-1 == 3
        boundedline(time_vector, mean_Rob_Ps_Aopt(1,:), SEM_Rob_Ps_Aopt(1,:)*ts, ...
            time_vector, mean_Rob_Ps_Aopt(2,:), SEM_Rob_Ps_Aopt(2,:)*ts, ...
            time_vector, mean_Rob_Ps_Aopt(3,:), SEM_Rob_Ps_Aopt(3,:)*ts, ...
            'alpha', 'cmap', colours, 'transparency', transparency);
    elseif length(methods)-1 == 2
        boundedline(time_vector, mean_Rob_Ps_Aopt(1,:), SEM_Rob_Ps_Aopt(1,:)*ts, ...
            time_vector, mean_Rob_Ps_Aopt(2,:), SEM_Rob_Ps_Aopt(2,:)*ts, ...
            'alpha', 'cmap', colours, 'transparency', transparency);
    end
    
    for i = 1:length(methods)-1
        Rob_P_Aopt = mean_Rob_Ps_Aopt(i,:);
        h(i) = plot(time_vector, Rob_P_Aopt, 'LineWidth', 1, 'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('Robot uncertainty @ A-opt');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YTick'       , 0:0.05:1, ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size, ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    rescale_axes(rescale_factor);
    axis([0 time_vector(end) 0 0.2])
    %   pbaspect(gca, [1 2 1])
    hold off
    
    %% Robot covariance trace (D-opt) %%
    subplot(2,3,5)
    hold on
    if length(methods)-1 == 3
        boundedline(time_vector, mean_Rob_Ps_Dopt(1,:), SEM_Rob_Ps_Dopt(1,:)*ts, ...
            time_vector, mean_Rob_Ps_Dopt(2,:), SEM_Rob_Ps_Dopt(2,:)*ts, ...
            time_vector, mean_Rob_Ps_Dopt(3,:), SEM_Rob_Ps_Dopt(3,:)*ts, ...
            'alpha', 'cmap', colours, 'transparency', transparency);
    elseif length(methods)-1 == 2
        boundedline(time_vector, mean_Rob_Ps_Dopt(1,:), SEM_Rob_Ps_Dopt(1,:)*ts, ...
            time_vector, mean_Rob_Ps_Dopt(2,:), SEM_Rob_Ps_Dopt(2,:)*ts, ...
            'alpha', 'cmap', colours, 'transparency', transparency);
    end
    
    for i = 1:length(methods)-1
        Rob_P_Dopt = mean_Rob_Ps_Dopt(i,:);
        h(i) = plot(time_vector, Rob_P_Dopt, 'LineWidth', 1, 'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('Robot uncertainty @ D-opt');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YTick'       , 0:10^-4:10^-2, ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size, ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    rescale_axes(rescale_factor);
    axis([0 time_vector(end) 0 0.5*10^-4])
    %   pbaspect(gca, [1 2 1])
    hold off
 
    %% Robot pose error %%
    subplot(2,3,6)
    hold on
    if length(methods)-1 == 3
        boundedline(time_vector, mean_pose_errors(1,:), SEM_pose_errors(1,:)*ts, ...
            time_vector, mean_pose_errors(2,:), SEM_pose_errors(2,:)*ts, ...
            time_vector, mean_pose_errors(3,:), SEM_pose_errors(3,:)*ts, ...
            'alpha', 'cmap', colours, 'transparency', transparency);
    elseif length(methods)-1 == 2
        boundedline(time_vector, mean_pose_errors(1,:), SEM_pose_errors(1,:)*ts, ...
            time_vector, mean_pose_errors(2,:), SEM_pose_errors(2,:)*ts, ...
            'alpha', 'cmap', colours, 'transparency', transparency);
    end
    
    for i = 1:length(methods)-1
        pose_error = mean_pose_errors(i,:);
        h(i) = plot(time_vector, pose_error, 'LineWidth', 1, 'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('Pose error (m)');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YTick'       , 0:0.05:0.2, ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size, ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    rescale_axes(rescale_factor);
    axis([0 time_vector(end) 0 0.2])
    %    pbaspect(gca, [1 2 1])
    hold off

    set(gcf, 'Position', [86, 540, 728, 434])
    
    if (do_print)
        fig = gcf;
        fig.PaperUnits = 'inches';
        fig.PaperPosition = paper_pos;
        fig.PaperPositionMode = 'manual';
        print(fig, '-depsc', [file_path, 'methods.eps']);
    end
    
    
    if (show_legend)
        h_legend = legend(h, 'No UI', 'UI - N = 5', 'UI - N = 9');
        %set(h_legend, 'Location', 'SouthOutside');
        %set(h_legend, 'orientation', 'horizontal')
        %set(h_legend, 'box', 'off')
    end
    
end

function [] = rescale_axes(scalefactor)

g = get(gca,'Position');
g(1:2) = g(1:2) + (1-scalefactor)/2*g(3:4);
g(3:4) = scalefactor*g(3:4);
set(gca,'Position',g);

end

%close all;