max_trace = 0;

for i = 1:200
    
    for j = 1:50
        
        try
            Rob_P_trace = trace(logger.(['trial', num2str(j)]).uncertainty_rate_UI_N_gauss_5.Rob_Ps(:,:,i));
        catch
            continue
        end
        
        if (Rob_P_trace > max_trace)
            max_trace = Rob_P_trace;
        end
    
    end
    
end

max_trace