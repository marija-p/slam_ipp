% Transfer data from logger1 to logger objects.

trials = fieldnames(logger1);
methods = fieldnames(logger1.trial18);

for i = 1:length(trials)
    for j = 1:length(methods)
        logger.(trials{i}).(methods{j}) = logger1.(trials{i}).(methods{j});
    end
end