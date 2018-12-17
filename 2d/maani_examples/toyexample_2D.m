clc; clear; close all
rng('shuffle')
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

% Cool colors
green = [0.2980 .6 0];
crimson = [220,20,60]/255; 
orange = [255,127,80]/255;
darkblue = [0 .2 .4];
Darkgrey = [.25 .25 .25];
darkgrey = [.35 .35 .35];
lightgrey = [.7 .7 .7];
Lightgrey = [.9 .9 .9];
VermillionRed = [156,31,46]/255;
DupontGray = [144,131,118]/255;
Azure = [0, 127, 255] / 255;

fontsize = 18; % font size

% find the Gauss-Hermite abscissae and weights
N = 9;
% [X, W] = hermquad(N);

%% GP
meanfunc = {@meanZero};
covfunc = {@covSEard};
likfunc = {@likGauss};
inffunc = {@infExact};
ell = 0.5; sf = rand; sn = 0.05;
hyp.mean = [];
hyp.cov = log([ell; ell; sf]);
hyp.lik = log(sn);

% training data
x = (-3:1:3)';
[X1, X2] = meshgrid(x,x);
X = [X1(:), X2(:)];
Y =  peaks(X1,X2);
% add noise to measurements
n = size(X,1);
yn = sn * rand(n,1);
y = Y(:) + yn;

% ground truth
xgt = (-3:.4:3)';
[Xgt1, Xgt2] = meshgrid(xgt,xgt);
Xgt = [Xgt1(:), Xgt2(:)];
Ygt = peaks(Xgt1, Xgt2);
% ygt = Ygt(:);
t = Xgt; % test data

% perturb input data
S2X = diag([0.6^2, 0.6^2]);
L = chol(S2X, 'lower');
% Xp = X + (L * randn(size(X))')';
load('Xp_2d.mat'); % a fixed sample using S2X = diag([0.6^2, 0.6^2])

% learn hyperparameters
hyp = minimize(hyp, @gp, -100, inffunc, meanfunc, covfunc, likfunc, Xp, y);

% inference using deterministic input
% [ymu, ys2, fmu, fs2, lp] = gp(hyp, inffunc, meanfunc, covfunc, likfunc, x, y, t);

% inference using uncertain input (ignoring uncertainty)
[ymu1, ys21, fmu1, fs21, lp1, post1] = gp(hyp, inffunc, meanfunc, covfunc, likfunc, Xp, y, t);


% covariance with uncertain input
cov2 = {@covUI, covfunc, N, S2X};
hyp2 = minimize(hyp, @gp, -100, inffunc, meanfunc, cov2, likfunc, Xp, y);
[ymu2, ys22, fmu2, fs22, lp2, post2] = gp(hyp2, inffunc, meanfunc, cov2, likfunc, Xp, y, t);


% plotting

figure;
pcolor(Xgt1, Xgt2, Ygt)
xlabel('$x_1$', 'Interpreter','latex', 'fontsize',fontsize)
ylabel('$x_2$', 'Interpreter','latex', 'fontsize',fontsize)
title('Ground truth', 'Interpreter','latex', 'fontsize',fontsize)
axis equal tight
colorbar, colormap jet
shading faceted
set(gca,'fontsize',fontsize)
set(gca,'TickLabelInterpreter','latex')
figuresize(21,21,'cm')
print -opengl -dpng -r300 gt_2d.png

figure;
pcolor(Xgt1, Xgt2, reshape(ymu1, size(Xgt1)))
xlabel('$x_1$', 'Interpreter','latex', 'fontsize',fontsize)
ylabel('$x_2$', 'Interpreter','latex', 'fontsize',fontsize)
title('GP - ignoring input uncertainty', 'Interpreter','latex', 'fontsize',fontsize)
axis equal tight
colorbar, colormap jet
shading faceted
set(gca,'fontsize',fontsize)
set(gca,'TickLabelInterpreter','latex')
figuresize(21,21,'cm')
print -opengl -dpng -r300 gp_ignoring_input_uncertainty_2d.png

figure;
pcolor(Xgt1, Xgt2, reshape(ymu2, size(Xgt1)))
xlabel('$x_1$', 'Interpreter','latex', 'fontsize',fontsize)
ylabel('$x_2$', 'Interpreter','latex', 'fontsize',fontsize)
title('GP - incorporating input uncertainty', 'Interpreter','latex', 'fontsize',fontsize)
axis equal tight
colorbar, colormap jet
shading faceted
set(gca,'fontsize',fontsize)
set(gca,'TickLabelInterpreter','latex')
figuresize(21,21,'cm')
print -opengl -dpng -r300 gp_incorporating_input_uncertainty_2d.png

%% Kernel behavior

% kernel space plots

K = post1.L * post1.L'; % SE kernel
Kui =  post2.L * post2.L'; % expected (modified) SE kernel

figure; hold on
figuresize(21,21,'cm')

subplot(1,2,1)
image(K,'CDataMapping','scaled')
axis equal
axis off
colorbar, colormap jet
shading faceted
% xlabel('x_1','fontsize',fontsize)
% ylabel('x_2','fontsize',fontsize)
title('\boldmath$K$', 'Interpreter','latex', 'fontsize',fontsize)
set(gca,'FontSize',fontsize)

% figure
subplot(1,2,2)
image(Kui,'CDataMapping','scaled')
axis equal
axis off
colorbar, colormap jet
shading faceted
% xlabel('x_1','fontsize',fontsize)
% ylabel('x_2','fontsize',fontsize)
title('Expected (modified) \boldmath$K$', 'Interpreter','latex', 'fontsize',fontsize)
set(gca,'FontSize',fontsize)

% print -opengl -dpng -r300 kernel_space_2d_0.6.png
