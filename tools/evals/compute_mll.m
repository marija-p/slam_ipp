function mll = compute_mll(field_map, ground_truth_map)
% p.23 of http://www.gaussianprocess.org/gpml/chapters/RW2.pdf
% Bayesian Opt. for Informative Continuous Path Planning - Marchant & Ramos (ICRA, 2014)
    
ll = (0.5.*log(2.*pi.*field_map.cov)) + ...
    (((ground_truth_map - field_map.mean).^2)./(2.*field_map.cov));
ll_sum = sum(ll);
mll = ll_sum/numel(field_map.mean);

end
