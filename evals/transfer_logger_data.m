% Transfer data from logger1 to logger objects.

trials = fieldnames(logger);
methods = fieldnames(logger1.trial1);

for i = 1:length(trials)
    for j = 2:length(methods)
        logger.(trials{i}).(methods{j}) = logger1.(trials{i}).(methods{j});
    end
end