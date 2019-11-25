% Transfer data from logger1 to logger objects.

trials = fieldnames(logger);

%trials = trials(1:50); % Environmnt 1
%trials = trials(51:99); % Environment 8
%trials = trials(100:149); % Environment 6
%trials = trials(150:199); % Environment 9
trials = trials(200:248); % Environment 10

methods = fieldnames(logger.trial31);

for i = 1:length(trials)
    for j = 1:length(methods)
        logger_final.(trials{i}).(methods{j}) = logger.(trials{i}).(methods{j});
    end
end