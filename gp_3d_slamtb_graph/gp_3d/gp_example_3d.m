load ground_truth_3d.mat

%% Hyperparameter training %%

% Training data.
X_train = mesh;
Y_train = ground_truth;

train_hyperparameters = 0;

inf_func = @infExact;
cov_func = @covSEiso; 
lik_func = @likGauss; sn = 0.1; hyp.lik = log(sn);
mean_func = @meanConst;

if (train_hyperparameters)
    hyp.cov = [0 ; 0];
    hyp.lik = log(0.1);
    hyp.mean = 24.3446;
    hyp_trained = ...
        minimize(hyp, @gp, -200, inf_func, mean_func, ...
        cov_func, lik_func, X_train, Y_train);
end

%% GP Regression %%
X_predict = X_train;
test_ind = datasample(1:size(X_predict,1),3);
X_test = X_train(test_ind,:);
Y_test = Y_train(test_ind);

% ymu, ys: mean and covariance for output
% fmu, fs: mean and covariance for latent variables
% post: struct representation of the (approximate) posterior
[ymu, ys, fmu, fs, ~ , post] = ...
    gp(hyp_trained, inf_func, mean_func, cov_func, lik_func, ...
    X_test, Y_test, X_predict);
Y_predict = ymu;

%% Plotting %%
subplot(1,4,1)
scatter3(X_train(:,1), X_train(:,2), X_train(:,3), 100, Y_train, 'filled');
title('Training data (GT)')
xlabel('x')
ylabel('y')
zlabel('z')
caxis([0 45])
subplot(1,4,2)
scatter3(X_test(:,1), X_test(:,2), X_test(:,3), 100, Y_test, 'filled');
title('Test (input)')
xlabel('x')
ylabel('y')
zlabel('z')
caxis([0 45])
subplot(1,4,3)
scatter3(X_predict(:,1), X_predict(:,2), X_predict(:,3), 100, Y_predict, 'filled');
title('Prediction')
xlabel('x')
ylabel('y')
zlabel('z')
caxis([0 45])
subplot(1,4,4)
scatter3(X_predict(:,1), X_predict(:,2), X_predict(:,3), 100, ys, 'filled');
title('Variance')
xlabel('x')
ylabel('y')
zlabel('z')

set(gcf, 'Position', [224, 639, 1021, 332]);