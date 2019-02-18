dim_x_env = 2.8;
dim_y_env = 2.8;

res_x = 0.4;
res_y = 0.4;

inf_func = @infExact;
cov_func = @covSEiso;
lik_func = @likGauss; sn = 0.1; hyp.lik = log(sn);
mean_func = @meanConst;

% Create grid of locations.
x = 0:res_x:dim_x_env;
y = 0:res_y:dim_y_env;
[X,Y] = meshgrid(x,y); mesh = [X(:) Y(:)];

% Training data.
X_train = mesh;
Y_train = reshape(temp_data, numel(temp_data), []);

% Train the model.
hyp.cov = [0 ; 0];
hyp.lik = log(0.1);
hyp.mean = 24;
hyp_trained = ...
    minimize(hyp, @gp, -200, inf_func, mean_func, ...
    cov_func, lik_func, X_train, Y_train);

disp(hyp_trained)