close all;

trials = fieldnames(logger);
planners = fieldnames(logger.trial1);

delete_last_figure = 0;

for i = 1:length(trials)
    figure;
    for j = 2:length(planners)
        if length(fieldnames(logger.(trials{i}))) == length(planners)
            plot_metrics(logger.(trials{i}).(planners{j}));
            legend(planners(2:end));
        else
            delete_last_figure = 1;
        end
    end
end

if (delete_last_figure)
    close;
end