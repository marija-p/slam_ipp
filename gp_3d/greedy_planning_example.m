clear all;
load training_data.mat

X_predict = X_train;
test_ind = datasample(1:size(X_predict,1),2);
X_test = X_train(test_ind,:);
Y_test = Y_train(test_ind);

num_iters = 5;

[ymu, ys, fmu, fs, ~ , post] = ...
    gp(hyp_trained, inf_func, [], cov_func, lik_func, ...
    X_test, Y_test, X_predict);

for i = 1:num_iters
    
    % Find most uncertain point.
    [~, point_next_ind] = max(ys);
    
    subplot(1,num_iters,i)
    hold on
    scatter3(X_predict(:,1), X_predict(:,2), X_predict(:,3), 100, ymu, 'filled');
    scatter3(X_train(point_next_ind,1), X_train(point_next_ind,2), ...
        X_train(point_next_ind,3), Y_train(point_next_ind), 200, 'r', 'filled')
    title(['[', num2str(X_train(point_next_ind,1),4), ', ', ...
        num2str(X_train(point_next_ind,2),4), ', ', ...
        num2str(X_train(point_next_ind,3),4), ']'])
    xlabel('x')
    ylabel('y')
    zlabel('z')
    caxis([0 45])
    view(3)
    hold off
    
    % Update test set with this point.
    X_test = [X_test; X_train(point_next_ind, :)];
    Y_test = [Y_test; Y_train(point_next_ind)];
    
    % Do GP regression.
    [ymu, ys, fmu, fs, ~ , post] = ...
        gp(hyp_trained, inf_func, [], cov_func, lik_func, ...
        X_test, Y_test, X_predict);
    
end

set(gcf, 'Position', [58, 690, 1182, 281])