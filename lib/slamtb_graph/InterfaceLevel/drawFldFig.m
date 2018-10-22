function FldFig = drawFldFig(FldFig, Rob, SimRob, ymu, ys)

% DRAWFIELDFIG  Redraw the field figure.

global Map

hold on
% FIELD MAP
FldFig.field_mean.CData = ymu;
FldFig.field_cov.CData = ys;

for k = 1:2
    
    % ESTIMATED OBJECTS
    % robots
    for rob = 1:numel(Rob)
        
        % create and draw robot - with ellipse
        FldFig.Rob(rob,k).patch = drawObject(FldFig.Rob(rob,k).patch,Rob(rob));
        if ~isempty(Rob(rob).state.r)
            r = Rob(rob).state.r(1:3);
            P = Map.P(r,r);
            drawEllipse(FldFig.Rob(rob,k).ellipse,Rob(rob).frame.x(1:3),2*P);
        end
        
    end
    
    % SIMULATED OBJECTS
    % robots
    for rob = 1:numel(SimRob)
        
        FldFig.simRob(rob,k) = drawObject(FldFig.simRob(rob,k),SimRob(rob));
        
    end
    
end