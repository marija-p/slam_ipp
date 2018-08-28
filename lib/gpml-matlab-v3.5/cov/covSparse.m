function K = covSparse(hyp, x, z, i)

% Squared Exponential covariance function with Automatic Relevance Detemination
% (ARD) distance measure. The covariance function is parameterized as:
%
% k(x^p,x^q) = sf^2 * exp(-(x^p - x^q)'*inv(P)*(x^p - x^q)/2)
%
% where the P matrix is diagonal with ARD parameters ell_1^2,...,ell_D^2, where
% D is the dimension of the input space and sf2 is the signal variance. The
% hyperparameters are:
%
% hyp = [ log(ell_1)
%         log(ell_2)
%          .
%         log(ell_D)
%         log(sf) ]
%
% Copyright (c) by Carl Edward Rasmussen and Hannes Nickisch, 2010-09-10.
%
% See also COVFUNCTIONS.M.

if nargin<2, K = '(D+1)'; return; end              % report number of parameters
if nargin<3, z = []; end                                   % make sure, z exists
xeqz = isempty(z); dg = strcmp(z,'diag');                       % determine mode

[n,D] = size(x);
ell = exp(hyp(1:D));                               % characteristic length scale
sf2 = exp(2*hyp(D+1));                                         % signal variance

% precompute squared distances
if dg                                                               % vector kxx
  R = zeros(size(x,1),1);
else
  if xeqz                                                 % symmetric matrix Kxx
    R = sqrt(sq_dist(diag(1./ell)*x'));
    zind = R>=1;
  else                                                   % cross covariances Kxz
    R = sqrt(sq_dist(diag(1./ell)*x',diag(1./ell)*z'));
    zind = R>=1;
  end
end

if nargin <= 3
    K = sf2* ((1/3) * (1-R).* (2+cos(2*pi*R)) + (1/(2*pi)) * sin(2*pi*R));    % covariance
    if ~dg
        K(zind) = 0;
    end
    K = sparse(K);
elseif nargin>3                                                        % derivatives
  if i<=D                                              % length scale parameters
    if dg
      K = R*0;
    else
      K = R(:);
      if xeqz
          for j=1:length(K)
              if K(j) < 1 && K(j) ~= 0 
                  K(j) = 4*sf2/3 * (pi*(1-K(j)) * cos(pi*K(j)) + sin(pi*K(j))) * sin(pi*K(j))/K(j);
              else
                  K(j) = 0;
              end
          end
          K = reshape(K,size(R)) .* sq_dist(x(:,i)'/ell(i)) /ell(i);
      else
          for j=1:length(K)
              if K(j) < 1 && K(j) ~= 0 
                  K(j) = 4*sf2/3 * (pi*(1-K(j)) * cos(pi*K(j)) + sin(pi*K(j))) * sin(pi*K(j))/K(j);
              else
                  K(j) = 0;
              end
          end
          K = reshape(K,size(R)) .* sq_dist(x(:,i)'/ell(i),z(:,i)'/ell(i)) /ell(i);
      end
      
    end
%     K(zind) = 0;
    K = sparse(K);
  elseif i==D+1                                            % magnitude parameter
        K = ((1/3) * (1-R).* (2+cos(2*pi*R)) + (1/(2*pi)) * sin(2*pi*R));    % covariance
        K(zind) = 0;
        K = sparse(K);
  else
    error('Unknown hyperparameter')
  end
end