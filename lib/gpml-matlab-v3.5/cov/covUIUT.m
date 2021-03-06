function K = covUIUT(cov, N, S2, hyp, x, z, i)

% A wraper for integration over covariance functions with uncertain input.
%
% The input data u is considered to be normally distributed N(x,S2).
% This function uses Gauss-Hermite quadrature to numerically compute the
% integral of the covariance function over N(x,S2) where x is
% 2-dimensional.
% N: number of sample points in Gauss-Hermite quadrature 
% S2: 2-dimensional covariance matrix for input data
%
% Maani Ghaffari Jadidi

if nargin < 5, K = feval(cov{:}); return, end
if nargin < 6, z = []; end                           % make sure, z exists
xeqz = isempty(z); dg = strcmp(z,'diag');            % determine mode

try
    L = cell(size(x,1));
    y = L;
    for i = 1:size(x,1)
        [y{i},Py] = ut_pose2map(zeros(3,1),S2,x(i,:));
        L{i} = chol(Py);
    end
    [X, W] = hermquad(N);
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
            for l = 1:N
                Wtot = W(j) * W(l);
                if abs(Wtot) > 1e-6
                    u = x;
                    for i = 1:size(x,1)
%                         [y,Py] = ut_pose2map(zeros(3,1),S2,x(i,:));
%                         L = chol(Py);
                        u(i,:) = L{i} * [X(j); X(l)] + y{i};
                    end
                    if xeqz
                        K = K + Wtot * feval(cov{:},hyp,u);
                    else
                        K = K + Wtot * feval(cov{:},hyp,u,z);
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
            for l = 1:N
                Wtot = W(j) * W(l);
                if abs(Wtot) > 1e-6
                    u = x;
                    for i = 1:size(x,1)
                        [y,Py] = ut_pose2map(zeros(3,1),S2,x(i,:));
                        L = chol(Py);
                        u(i,:) = L * [X(j); X(l)] + y;
                    end
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
if ~dg
    K = K / (2*pi);
end