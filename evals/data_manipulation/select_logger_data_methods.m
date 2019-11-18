% Create logger object with only select methods.

%logger1 = logger;

trials = fieldnames(logger1);
%methods = fieldnames(logger.trial32);
methods = {'random', 'rig'};

%logger = [];

for i = 1:length(trials)
    for j = 1:length(methods)
        logger.(trials{i}).(methods{j}) = logger1.(trials{i}).(methods{j});
    end
end