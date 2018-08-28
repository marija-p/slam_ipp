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
hyp.cov = log([ell; sf]);
hyp.lik = log(sn);

% training data
f = @(z) .2*cos(z.^2).*exp(-z) + .15*sin(z);

x = (1:1.7:7)';
n = length(x);
yn = sn*rand(n,1);
y = f(x); y = y + yn;

% ground truth
xgt = (0:.1:7)';
ygt = f(xgt);
t = xgt; % test data

% perturb input data
s2x = 0.6^2;
% xp = x + sqrt(s2x) * randn(size(x));
xp = [1.0238; 2.5131; 4.1915; 5.9266]; % a fixed sample using s2x = 0.6^2

% learn hyperparameters
hyp = minimize(hyp, @gp, -100, inffunc, meanfunc, covfunc, likfunc, xp, y);

% inference using deterministic input
% [ymu, ys2, fmu, fs2, lp] = gp(hyp, inffunc, meanfunc, covfunc, likfunc, x, y, t);

% inference using uncertain input (ignoring uncertainty)
[ymu1, ys21, fmu1, fs21, lp1, post1] = gp(hyp, inffunc, meanfunc, covfunc, likfunc, xp, y, t);


% covariance with uncertain input
cov2 = {@covUIone, covfunc, N, s2x};
hyp2 = minimize(hyp, @gp, -100, inffunc, meanfunc, cov2, likfunc, xp, y);
[ymu2, ys22, fmu2, fs22, lp2, post2] = gp(hyp2, inffunc, meanfunc, cov2, likfunc, xp, y, t);


% plotting
h = [];
figure; hold on
set(gca,'fontsize',fontsize)
set(gca,'TickLabelInterpreter','latex')

h{1} = [ymu1+2*sqrt(ys21); flipdim(ymu1-2*sqrt(ys21),1)]; 
fill([t; flipdim(t,1)], h{1}, orange, 'LineStyle','none', 'FaceAlpha', .3)

h{2} = [ymu2+2*sqrt(ys22); flipdim(ymu2-2*sqrt(ys22),1)]; 
fill([t; flipdim(t,1)], h{2}, Azure, 'LineStyle','none', 'FaceAlpha', .2)

h{3} = plot(xgt, ygt, '--', 'linewidth', 3, 'color', DupontGray);
h{4} = plot(t, ymu1, '-.', 'linewidth', 2, 'color', orange);
h{5} = plot(t, ymu2, '-', 'linewidth', 2, 'color', Azure);
h{6} = plot(xp,y,'.', 'markersize', 18, 'color', 'k');

legend([h{3},h{4},h{5},h{6}], {'Groundtruth','GP', 'GP-EK','Training points'},'location','best')


xlabel('Input, \boldmath$x$', 'Interpreter','latex', 'fontsize',fontsize)
ylabel('Output, $f$(\boldmath$x$)', 'Interpreter','latex', 'fontsize',fontsize)
axis tight
figuresize(21,12,'cm')
% print -opengl -dpng -r300 gp_ui_sigma_0.6.png


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

% print -opengl -dpng -r300 kernel_space_sigma_0.6.png
