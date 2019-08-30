% Transfer data from logger1 to logger objects.

trials = fieldnames(logger2);
methods = fieldnames(logger2.trial31);

for i = 1:length(trials)
    for j = 1:length(methods)
        logger.(trials{i}).(methods{j}) = logger2.(trials{i}).(methods{j});
    end
end