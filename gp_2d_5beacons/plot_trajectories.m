figure;

subplot(1,2,1)
hold on
scatter(X_train(100:200,1), X_train(100:200,2), 50, Y_train(100:200), 'filled');
for i = 100:5:200
    draw_ellipse(X_train(i,:)',P_train(:,:,i),3);
end
grid minor
colormap hot
caxis([0 50])
axis([-10, 50, -10, 50])
title('Estimated traj. ')

subplot(1,2,2)
scatter(X_train_gt(:,1), X_train_gt(:,2), 50, Y_train, 'filled');
grid minor
colormap hot
caxis([0 50])
axis([-10, 50, -10, 50])
title('Real traj. ')

set(gcf, 'Position', [-381, 905, 1017, 404]);

function eH = draw_ellipse(x,P,nSigma)
eH = [];
if(~any(diag(P)==0))
    [V,D] = eig(P);
    y = nSigma*[cos(0:0.1:2*pi);sin(0:0.1:2*pi)];
    el = V*sqrtm(D)*y;
    el = [el el(:,1)]+repmat(x,1,size(el,2)+1);
    eH = line(el(1,:),el(2,:),'Color','k','LineWidth',1);
end

end