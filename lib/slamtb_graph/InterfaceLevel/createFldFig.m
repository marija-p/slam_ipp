function FldFig = createFldFig(Rob,SimRob,X_test,FigOpt)
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
    'Position',      [-1385         637        1053         415]);


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
FldFig.field_mean = scatter(X_test(:,1), X_test(:,2), 80, zeros(size(X_test,1),1), ...
    'filled');
caxis([0 50])
colorbar
title('Mean')

subplot(1,2,2)
hold on
FldFig.field_cov = scatter(X_test(:,1), X_test(:,2), 80, zeros(size(X_test,1),1), ...
    'filled');
caxis([10 180])
colorbar
title('Variance')

end