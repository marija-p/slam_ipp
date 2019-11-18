% Transfer data from logger1 to logger objects.

trials = fieldnames(logger1);
methods = fieldnames(logger1.trial1);

num_new_trial = 200;

for i = 1:length(trials)
    for j = 1:length(methods)
        logger.(['trial', num2str(num_new_trial)]).(methods{j}) = ...
            logger1.(trials{i}).(methods{j});
        logger.(['trial', num2str(num_new_trial)]).('num') = num_new_trial; 
    end
    num_new_trial = num_new_trial + 1;
end