function K = covUIMCone(cov, N, s2x, hyp, x, z, i)

% A wraper for integration over covariance functions with uncertain input
% using Monte Carlo integration.
% The input data u is considered to be normally distributed N(x,s2x).
% This function uses Monte Carlo integration to numerically compute the
% integral of the covariance function over N(x,s2x) where x is
% 1-dimensional.
% N: number of sample points in Monte Carlo integration 
% s2x: 1-dimensional variance value for input data
%
% Maani Ghaffari Jadidi

if nargin < 5, K = feval(cov{:}); return, end
if nargin < 6, z = []; end                           % make sure, z exists
xeqz = isempty(z); dg = strcmp(z,'diag');            % determine mode

if ~(s2x > 0)
    if nargin < 7
        K = feval(cov{:}, hyp, x, z);
    else
        K = feval(cov{:}, hyp, x, z, i);
    end
    return
else
%     [X, W] = hermquad(N);
    [n, D] = size(x);
end

K = 0;
if nargin < 7                                                % covariances
    if dg
        K = feval(cov{:},hyp,x,'diag');
    else
        for j = 1:N
            u = sqrt(s2x) * randn(n,1) + x;
            if xeqz
                K = K + feval(cov{:},hyp,u);
            else
                K = K + feval(cov{:},hyp,u,z);
            end
        end
    end
else                                                         % derivatives
    if dg
        K = feval(cov{:},hyp,x,'diag',i);
    else
        for j = 1:N
            u = sqrt(s2x) * randn(n,1) + x;
            if xeqz
                K = K + feval(cov{:},hyp,u,[],i);
            else
                K = K + feval(cov{:},hyp,u,z,i);
            end
        end
    end
end
if ~dg
    K = K / N;
end