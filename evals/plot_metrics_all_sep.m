close all;

trials = fieldnames(logger);
methods = fieldnames(logger.trial1);

% Last trial is incomplete.
if (length(methods) ~= length(fieldnames(logger.(trials{end}))))
    trials = trials(1:end-1);
end

for i = 1:length(trials)
    figure;
    for j = 2:length(methods)
        plot_metrics(logger.(trials{i}).(methods{j}));
        legend(methods(2:end));
    end
end