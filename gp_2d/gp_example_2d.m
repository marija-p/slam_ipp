load ground_truth_2d.mat

rng(2)

%% Hyperparameter Training %%
train_hyperparameters = 1;

% Training data.
X_gt = mesh;
Y_gt = ground_truth;

% Number of training points (input).
N_train_points = 20;

% Number of sample points in Gauss Hermite quadrature.
N_gauss = 11;
% Uncertainty on location input (covariance matrix).
S2X = diag([1^2, 1^2]);

% GP hyperparameters.
mean_func = {@meanConst};
inf_func = {@infExact};
cov_func = {@covSEiso};
cov_func_UI = {@covUI, cov_func, N_gauss, S2X};
lik_func = @likGauss; sn = 0.1; hyp.lik = log(sn);

if (train_hyperparameters)
    hyp.cov = [0; 0];
    hyp.lik = log(0.1);
    hyp.mean = 25.5421;
    hyp_trained = ...
        minimize(hyp, @gp, -100, inf_func, mean_func, ...
        cov_func, lik_func, X_gt, Y_gt);
    
    hyp_trained_UI = ...
        minimize(hyp, @gp, -100, inf_func, mean_func, ...
        cov_func_UI, lik_func, X_gt, Y_gt);
end


%% GP Regression %%
X_test = X_gt;
test_ind = datasample(1:size(X_test,1),N_train_points);
X_train = X_gt(test_ind,:);
Y_train = Y_gt(test_ind);

% add noise to measurements
n = size(X_train,1);
yn = log(0.1) * rand(n,1);
%Y_train = Y_train + yn;

% ymu, ys: mean and covariance for output
% fmu, fs: mean and covariance for latent variables
% post: struct representation of the (approximate) posterior

% With certain input.
[ymu1, ys1, fmu1, fs1, ~ , post1] = ...
    gp(hyp_trained, inf_func, mean_func, cov_func, lik_func, ...
    X_train, Y_train, X_test);

% With uncertain input.
[ymu2, ys2, fmu2, fs2, ~ , post2] = ...
    gp(hyp_trained, inf_func, mean_func, cov_func_UI, lik_func, ...
    X_train, Y_train, X_test);


%% Plotting %%

% Visualize regression results.
subplot(3,2,1)
scatter(X_gt(:,1), X_gt(:,2), 100, Y_gt, 'filled');
title('Ground truth')
xlabel('x')
ylabel('y')
zlabel('z')
caxis([0 45])
colorbar

subplot(3,2,2)
scatter(X_train(:,1), X_train(:,2), 100, Y_train, 'filled');
title('Training data (Input)')
xlabel('x')
ylabel('y')
zlabel('z')
caxis([0 45])
colorbar

subplot(3,2,3)
scatter(X_test(:,1), X_test(:,2), 100, ymu1, 'filled');
title('Prediction - w/o uncertainty')
xlabel('x')
ylabel('y')
zlabel('z')
caxis([0 45])
colorbar

subplot(3,2,4)
scatter(X_test(:,1), X_test(:,2), 100, ys1, 'filled');
title('Variance - w/o uncertainty')
xlabel('x')
ylabel('y')
zlabel('z')
caxis([0 100])
colorbar

subplot(3,2,5)
scatter(X_test(:,1), X_test(:,2), 100, ymu2, 'filled');
title('Prediction - w/ uncertainty')
xlabel('x')
ylabel('y')
zlabel('z')
caxis([0 45])
colorbar

subplot(3,2,6)
scatter(X_test(:,1), X_test(:,2), 100, ys2, 'filled');
title('Variance - w/ uncertainty')
xlabel('x')
ylabel('y')
zlabel('z')
caxis([0 100])
colorbar

% Visualize covariance matrices.
figure;
subplot(1,2,1)
L = post1.L;
sW = post1.sW;
Kss = real(feval(cov_func{:}, hyp_trained.cov, X_test));
Ks = feval(cov_func{:}, hyp_trained.cov, X_train, X_test);
Lchol = isnumeric(L) && all(all(tril(L,-1)==0)&diag(L)'>0&isreal(diag(L))');
if Lchol    % L contains chol decomp => use Cholesky parameters (alpha,sW,L)
    V = L'\(sW.*Ks);
    K = Kss - V'*V;                       % predictive variances
else                % L is not triangular => use alternative parametrisation
    if isnumeric(L), LKs = L*(Ks); else LKs = L(Ks); end    % matrix or callback
    K = Kss + Ks'*LKs;                    % predictive variances
end
imagesc(K)
title('W/o uncertainty')
colorbar
caxis([-30, 90])


subplot(1,2,2)
L = post2.L;
sW = post2.sW;
Kss = real(feval(cov_func_UI{:}, hyp_trained.cov, X_test));
Ks = feval(cov_func_UI{:}, hyp_trained.cov, X_train, X_test);
Lchol = isnumeric(L) && all(all(tril(L,-1)==0)&diag(L)'>0&isreal(diag(L))');
if Lchol    % L contains chol decomp => use Cholesky parameters (alpha,sW,L)
    V = L'\(sW.*Ks);
    K_UI = Kss - V'*V;                       % predictive variances
else                % L is not triangular => use alternative parametrisation
    if isnumeric(L), LKs = L*(Ks); else LKs = L(Ks); end    % matrix or callback
    K_UI = Kss + Ks'*LKs;                    % predictive variances
end
imagesc(K_UI)
title('W/ uncertainty')
colorbar
caxis([-30, 90])


set(gcf, 'Position', [-1032, 196, 856, 907]);