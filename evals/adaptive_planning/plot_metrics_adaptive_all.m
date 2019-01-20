rescale_factor = 1;
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
    
time_vector = 0:0.1:200;

P_traces = zeros(length(methods)-1,length(time_vector));
P_traces_interesting = zeros(length(methods)-1,length(time_vector));
rmses = zeros(length(methods)-1,length(time_vector));
rmses_interesting = zeros(length(methods)-1,length(time_vector));

for i = 1:length(trials)
    
    for j = 2:length(methods)
        
        try
            time = logger.(trials{i}).(methods{j}).times;
        catch
            disp(['Cant find ', trials{i}, ' ' methods{j}])
            break;
        end
        
        P_trace = logger.(trials{i}).(methods{j}).P_traces;
        P_trace_interesting = logger.(trials{i}).(methods{j}).P_traces_interesting;
        rmse = logger.(trials{i}).(methods{j}).rmses;
        rmse_interesting = logger.(trials{i}).(methods{j}).rmses_interesting;
        
        ts = timeseries(P_trace, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        P_traces(j-1,:,i) = ts_resampled.data';
        
        ts = timeseries(P_trace_interesting, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        P_traces_interesting(j-1,:,i) = ts_resampled.data';
        
        ts = timeseries(rmse, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        rmses(j-1,:,i) = ts_resampled.data';
        
        ts = timeseries(rmse_interesting, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        rmses_interesting(j-1,:,i) = ts_resampled.data';
        
    end
    
end

% Find means and medians.
mean_P_traces = sum(P_traces,3)./length(trials);
mean_P_traces_interesting = sum(P_traces_interesting,3)./length(trials);
mean_rmses = sum(rmses,3)./length(trials);
mean_rmses_interesting = sum(rmses_interesting,3)./length(trials);

% Find confidence intervals
% http://ch.mathworks.com/matlabcentral/answers/159417-how-to-calculate-the-confidence-interva
SEM_P_traces = [];
SEM_P = [];
SEM_rmses = [];
SEM_rmses_interesting = [];

for j = 2:length(methods)
    
    SEM_P_traces(j-1,:) = std(squeeze(P_traces(j-1,:,:))', 'omitnan')/...
        sqrt(length(trials));
    SEM_P_traces_interesting(j-1,:) = (std(squeeze(P_traces_interesting(j-1,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    SEM_rmses(j-1,:) = (std(squeeze(rmses(j-1,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    SEM_rmses_interesting(j-1,:) = (std(squeeze(rmses_interesting(j-1,:,:))', 'omitnan')/...
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
    subplot(2,2,1)
    hold on
    h = zeros(length(methods)-1,1);
    boundedline(time_vector, mean_P_traces(1,:), SEM_P_traces(1,:)*ts, ...
        time_vector, mean_P_traces(2,:), SEM_P_traces(2,:)*ts, ...
        time_vector, mean_P_traces(3,:), SEM_P_traces(3,:)*ts, ...
        time_vector, mean_P_traces(4,:), SEM_P_traces(4,:)*ts, ...
        'alpha', 'cmap', colours, 'transparency', transparency);
    
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

    %% GP field covariance trace - interesting areas %%
    subplot(2,2,2)
    hold on
    h = zeros(length(methods)-1,1);
    boundedline(time_vector, mean_P_traces_interesting(1,:), SEM_P_traces_interesting(1,:)*ts, ...
        time_vector, mean_P_traces_interesting(2,:), SEM_P_traces_interesting(2,:)*ts, ...
        time_vector, mean_P_traces_interesting(3,:), SEM_P_traces_interesting(3,:)*ts, ...
        time_vector, mean_P_traces_interesting(4,:), SEM_P_traces_interesting(4,:)*ts, ...
        'alpha', 'cmap', colours, 'transparency', transparency);
    
    for i = 1:length(methods)-1
        P_trace_interesting = mean_P_traces_interesting(i,:);
        h(i) = plot(time_vector, P_trace_interesting, 'LineWidth', 1, 'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('Tr(P) - interesting');
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
    subplot(2,2,3)
    hold on
    boundedline(time_vector, mean_rmses(1,:), SEM_rmses(1,:)*ts, ...
        time_vector, mean_rmses(2,:), SEM_rmses(2,:)*ts, ...
        time_vector, mean_rmses(3,:), SEM_rmses(3,:)*ts, ...
        time_vector, mean_rmses(4,:), SEM_rmses(4,:)*ts, ...
        'alpha', 'cmap', colours, 'transparency', transparency);
    
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
    
    %% RMSE - interesting %%
    subplot(2,2,4)
    hold on
    boundedline(time_vector, mean_rmses_interesting(1,:), SEM_rmses_interesting(1,:)*ts, ...
        time_vector, mean_rmses_interesting(2,:), SEM_rmses_interesting(2,:)*ts, ...
        time_vector, mean_rmses_interesting(3,:), SEM_rmses_interesting(3,:)*ts, ...
        time_vector, mean_rmses_interesting(4,:), SEM_rmses_interesting(4,:)*ts, ...
        'alpha', 'cmap', colours, 'transparency', transparency);
    
    for i = 1:length(methods)-1
        rmse_interesting = mean_rmses_interesting(i,:);
        h(i) = plot(time_vector, rmse_interesting, 'LineWidth', 1, 'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('RMSE - interesting');
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
    
    set(gcf, 'Position', [-250, 654, 734, 485])
    
    if (do_print)
        fig = gcf;
        fig.PaperUnits = 'inches';
        fig.PaperPosition = paper_pos;
        fig.PaperPositionMode = 'manual';
        print(fig, '-depsc', [file_path, 'methods.eps']);
    end
    
    
    if (show_legend)
        h_legend = legend(h, 'No UI', 'No UI - adaptive', 'UI (N=9)', 'UI (N=9) - adaptive');
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