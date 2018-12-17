function K = covUIMC(cov, N, S2, hyp, x, z, i)

% A wraper for integration over covariance functions with uncertain input
% using Monte Carlo integration.
% The input data u is considered to be normally distributed N(x,S2).
% This function uses Monte Carlo integration to numerically compute the
% integral of the covariance function over N(x,S2) where x is
% D-dimensional.
% N: number of sample points in Monte Carlo integration 
% S2: D-dimensional covariance matrix for input data
%
% Maani Ghaffari Jadidi

if nargin < 5, K = feval(cov{:}); return, end
if nargin < 6, z = []; end                           % make sure, z exists
xeqz = isempty(z); dg = strcmp(z,'diag');            % determine mode

try
    L = chol(S2);
    [n, D] = size(x);
catch
    warning('Cholesky factorization of the input covariance matrix failed. Ignoring the uncertainties')
    if nargin < 7
        K = feval(cov{:}, hyp, x, z);
    else
        K = feval(cov{:}, hyp, x, z, i);
    end
    return    
end

K = 0;
if nargin < 7                                                % covariances
    if dg
        K = feval(cov{:},hyp,x,'diag');
    else
        for j = 1:N
            v =  L' * randn(D,n);
            u = v' + x;
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
            v =  L' * randn(D,n);
            u = v' + x;
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