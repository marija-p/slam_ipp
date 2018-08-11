close all;

% Function to generate 2-D Gaussian random field.

% Environment parameters.
dim_x_env = 60;
dim_y_env = 60;
res_x = 2;
res_y = 2;

% Correlation function parameters.
corr.name = 'gauss';
corr.c0 = [80, 80];
corr.sigma = 100;

% Create the random field.
x = linspace(0,dim_x_env,dim_x_env/res_x);
y = linspace(0,dim_y_env,dim_y_env/res_y);
[X,Y] = meshgrid(x,y); mesh = [X(:) Y(:)];
F = randomfield(corr,mesh);
% Scale.
F = F - min(F);

% Visualize the random field.
scatter(mesh(:,1), mesh(:,2), 100, F, 'filled')
h_cb = colorbar;
ylabel(h_cb, 'Temp. (deg)')
axis equal
xlabel('x (m)')
ylabel('y (m)')