function FldFig = drawFldFig(FldFig, Rob, SimRob, X_test, ymu, ys, FigOpt)

% DRAWFIELDFIG  Redraw the field figure.

global Map

hold on

for k = 1:2
    
    % ESTIMATED OBJECTS
    % robots
    for rob = 1:numel(Rob)
        
        % create and draw robot - with ellipse
        FldFig.Rob(rob,k).patch = drawObject(FldFig.Rob(rob,k).patch,Rob(rob));
        r = Rob(rob).state.r(1:3);
        drawEllipse(FldFig.Rob(rob,k).ellipse,Map.x(r),Map.P(r,r));
        
    end
    
    % SIMULATED OBJECTS
    
    % for each robot:
    for rob = 1:numel(SimRob)
        
        FldFig.simRob(rob,k) = drawObject(FldFig.simRob(rob,k),SimRob(rob));
        
    end
    
end

% FIELD MAP
FldFig.field_mean.CData = ymu;
FldFig.field_cov.CData = ys;