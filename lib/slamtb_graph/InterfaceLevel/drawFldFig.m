function FldFig = drawFldFig(FldFig, Rob, Lmk, SimRob, ymu, ys, FigOpt)

% DRAWFIELDFIG  Redraw the field figure.

global Map

hold on
% FIELD MAP
FldFig.field_mean.CData = ymu;
FldFig.field_cov.CData = ys;

% erase non used landmarks
used  = [Lmk.used];
drawn = [FldFig.Lmk(:,1).drawn];
erase = drawn & ~used;

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
    
    if any(erase)
        [FldFig.Lmk(erase,k).drawn] = deal(false);
        set([FldFig.Lmk(erase,k).mean],   'visible','off');
        set([FldFig.Lmk(erase,k).ellipse],'visible','off');
        set([FldFig.Lmk(erase,k).label],  'visible','off');
    end
    
    % for each landmark:
    for lmk=find(used)
        FldFig.Lmk(lmk,k).drawn = true;
        drawLmk(FldFig,Lmk(lmk),FigOpt.map);
    end
    
    % SIMULATED OBJECTS
    % robots
    for rob = 1:numel(SimRob)
        
        FldFig.simRob(rob,k) = drawObject(FldFig.simRob(rob,k),SimRob(rob));
        
    end
    
end