close all;

trials = fieldnames(logger);
methods = fieldnames(logger.trial1);

delete_last_figure = 0;

for i = 1:length(trials)
    figure;
    for j = 2:length(methods)
        if (length(fieldnames(logger.(trials{i}))) == length(methods))
            plot_metrics(logger.(trials{i}).(methods{j}));
            legend(methods(2:end));
        % Incomplete trial.
        else
            delete_last_figure = 1;
        end
    end
end

if (delete_last_figure)
    close;
end