function K = covUI_3d(cov, N, S2, hyp, x, z, i)

% A wrapper for integration over covariance functions with uncertain input.
%
% The input data u is considered to be normally distributed N(x,S2).
% This function uses Gauss-Hermite quadrature to numerically compute the
% integral of the covariance function over N(x,S2) where x is
% 2-dimensional.
% N: number of sample points in Gauss-Hermite quadrature
% S2: 3-dimensional covariance matrix for input data
%
% Maani Ghaffari Jadidi
% Marija Popovic

if nargin < 5, K = feval(cov{:}); return, end
if nargin < 6, z = []; end                           % make sure, z exists
xeqz = isempty(z); dg = strcmp(z,'diag');            % determine mode

try
    L = chol(S2);
    [X, W] = hermquad(N);
catch
    warning('Cholesky factorization of the input covariance matrix failed.Ignoring the uncertainties')
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
            for k = 1:N
                for l = 1:N
                    Wtot = W(j) * W(k) * W(l);
                    if abs(Wtot) > 1e-8
                        v =  L * [X(j); X(k); X(l)];
                        u = repmat(v',size(x,1),1) + x;
                        if xeqz
                            K = K + Wtot * feval(cov{:},hyp,u);
                        else
                            K = K + Wtot * feval(cov{:},hyp,u,z);
                        end
                    end
                end
            end
        end
    end
else                                                         % derivatives
    if dg
        K = feval(cov{:},hyp,x,'diag',i);
    else
        for j = 1:N
            for k = 1:N
                for l = 1:N
                    Wtot = W(j) * W(k) * W(l);
                    if abs(Wtot) > 1e-8
                        v =  L * [X(j); X(k); X(l)];
                        u = repmat(v',size(x,1),1) + x;
                        if xeqz
                            K = K + Wtot * feval(cov{:},hyp,u,[],i);
                        else
                            K = K + Wtot * feval(cov{:},hyp,u,z,i);
                        end
                    end
                end
            end
        end
    end
end
if ~dg
    K = K / (2*pi)^(3/2);
end