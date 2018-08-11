% startup script to make Octave/Matlab aware of the GPML package
%
% Copyright (c) by Carl Edward Rasmussen and Hannes Nickisch 2018-05-17.

disp('executing gpml startup script...')
mydir = fileparts(mfilename('fullpath'));                   % where am I located
addpath(mydir), dirs = {'cov','doc','inf','lik','mean','prior','util'};
for d = dirs, addpath([mydir,'/',d{:}]), end
addpath([mydir,'/util/sparseinv'])