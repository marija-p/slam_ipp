function FldFig = createFldFig(Rob,Lmk,SimRob,X_test,FigOpt)
% CREATEFLDFIG Create field figure and handles.

FldFig.fig = figure(5);
clf
hold on
set(FldFig.fig,...
    'numbertitle',   'off',...
    'name',          'Field Mapping',...
    'doublebuffer',  'off',...
    'renderer',      FigOpt.renderer,...
    'toolbar',       'none',...
    'color',         FigOpt.map.colors.bckgnd, ...
    'Position',      [639   535   532   227]);

% Axes
for k = 1:2
    subplot(1,2,k)
    FldFig.axes(k) = gca;
    axis equal
    set(FldFig.axes(k),...
        'parent',              FldFig.fig,...
        'xlim',                [FigOpt.map.lims.xMin FigOpt.map.lims.xMax],...
        'ylim',                [FigOpt.map.lims.yMin FigOpt.map.lims.yMax]);
    grid minor;
    xlabel('x (m)')
    ylabel('y (m)')
    daspect([1 1 0.5])
    axis([min(X_test(:,1)) max(X_test(:,1)) ...
        min(X_test(:,2)) max(X_test(:,2)) ...
        min(X_test(:,3)) max(X_test(:,3))])
    % 2D/3D plot orientation
    if (size(X_test,2) == 2)
        view(2)
    elseif (size(X_test,2) == 3)
        view(3)
    end
    
    % ESTIMATED OBJECTS
    % robots
    for rob = 1:numel(Rob)
        
        % create and draw robot - with ellipse
        FldFig.Rob(rob,k).patch = createObjPatch(Rob(rob),FigOpt.map.colors.est,FldFig.axes(k));
        FldFig.Rob(rob,k).ellipse = line(...
            'parent', FldFig.axes(k),...
            'xdata',  [],    ...
            'ydata',  [],    ...
            'zdata',  [],    ...
            'color',  'r',   ...
            'marker', 'none');
        
    end
    
    % landmarks
    for lmk = 1:numel(Lmk)
        
        FldFig.Lmk(lmk,k) = createLmkGraphics(Lmk(lmk),FigOpt.map.colors.label,FldFig.axes(k));
        
    end
    
    % SIMULATED OBJECTS
    if ~isempty(SimRob)
        
        % robots
        for rob = 1:numel(SimRob)
            
            % create and draw robot
            FldFig.simRob(rob,k) = createObjPatch(SimRob(rob),FigOpt.map.colors.simu,FldFig.axes(k));
            
        end
    end
    
end

% FIELD OBJECTS
% Initialise with zero values.
subplot(1,2,1)
hold on
% 2D/3D data
if (size(X_test,2) == 2)
    FldFig.field_mean = scatter(X_test(:,1), X_test(:,2), ...
        80, zeros(size(X_test,1),1), 'filled');
elseif (size(X_test,2) == 3)
    FldFig.field_mean = scatter3(X_test(:,1), X_test(:,2), X_test(:,3), ...
        10, zeros(size(X_test,1),1), 'filled');
end
caxis([0 50])
colorbar
title('Mean')
%FldFig.field_mean.MarkerFaceAlpha = 0.75;

subplot(1,2,2)
hold on
% 2D/3D data
if (size(X_test,2) == 2)
    FldFig.field_cov = scatter(X_test(:,1), X_test(:,2), ...
        80, zeros(size(X_test,1),1), 'filled');
    caxis([10 180])
elseif (size(X_test,2) == 3)
    FldFig.field_cov = scatter3(X_test(:,1), X_test(:,2), X_test(:,3), ...
        10, zeros(size(X_test,1),1), 'filled');
    caxis([0 500])
end
colorbar
title('Variance')
%FldFig.field_cov.MarkerFaceAlpha = 0.75;

end