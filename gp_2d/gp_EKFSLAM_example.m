load training_data_2d.mat

X_predict = X_train;
X_test = [];
X_test_gt = [];
Y_test = [];

% Global variables (avoid extra copies).
global xVehicleTrue;
global LandFeatures;
global LaserSensorSettings;
global VisionSensorSettings; % For incorporating later.
global xOdomLast;
global nSteps;
global UTrue;

% Setting up sensor specs.
LaserSensorSettings.Bearing = 30; % Degrees
LaserSensorSettings.Range = 50; % Meters
display(sprintf('=== Sensor Definitions ===  \n Max Range:  %d Meters\n Max Bearing: %d Degrees \n========================== \n',LaserSensorSettings.Range,LaserSensorSettings.Bearing));
display(' Paused!!! Press any Key!! ');
pause;

% Length of the simulation (number of time-steps).
nSteps = 600;

% Map size.
WorldSize = 200;

% Number of landmarks to add to the map.
nLandFeatures =5;
LandFeatures = zeros(2,1,nLandFeatures);

% Place beacons at the desired positions.
LandFeatures(:,:,1)=[0 10]';
LandFeatures(:,:,2)=[0 50]';
LandFeatures(:,:,3)=[50 50]';
LandFeatures(:,:,4)=[50 10]';
LandFeatures(:,:,5)=[40 40]';

% Initialization.
xVehicleTrue = [40 25 0]'; % start position (x, y, theta)
xEst =xVehicleTrue;
PEst = diag([1 1 0.01]);

% Array of detected landmarks.
MappedLandFeatures = NaN*zeros(nLandFeatures,2);

% Additive noise to be added at the observations.
obsNoise = [0; 0]; %randn(2,1);

% Plotting: Where landmarks are located in the world.
figure(1); hold on;
%grid off;
grid minor;
%axis equal;
colormap hot
plot(LandFeatures(1,:),LandFeatures(2,:),'b+');hold on;
set(gcf,'doublebuffer','on');
hLine = line([0,0],[0,0]);
set(hLine,'linestyle',':');
%axis([-WorldSize/4 WorldSize/2 -WorldSize/4 WorldSize/2]);
axis([-10 70 -10 70]);
xlabel(' Initial Conditions and beacons at blue +');
display(sprintf('\n\n\n Showing initial Location \n Press any key to procced\n'));
pause;

% Standard deviation errors.
UTrue = diag([0.01,0.01,1.5*pi/180]).^2; % Control.
RTrue = diag([1.1,5*pi/180]).^2; % Observation.


% Additive factor for control estimation and noise.
UEst = 0*UTrue;
REst = 0.2*RTrue;

% Initialize odometry measurement.
CtrlNoise = [0; 0; 0]; %randn(3,1); % some random control noise to be added
xOdomLast = GetOdometry(CtrlNoise);

for k = 2:nSteps
    
    CtrlNoise = [0; 0; 0]; %randn(3,1); % some random control noise to be added
    SimulateMovement(CtrlNoise);
    
    % Refresh odometry vector.
    xOdomNow = GetOdometry(CtrlNoise);
    u = tcomp(tinv(xOdomLast),xOdomNow);
    xOdomLast = xOdomNow;
    
    % Vehicle estimate.
    xVehicle = xEst(1:3);
    % Landmarks estimate.
    xMap = xEst(4:end);
    
    
    % Kalman prediction.
    xVehiclePred = tcomp(xVehicle,u);
    
    PPredvv = J1(xVehicle,u)* PEst(1:3,1:3) *J1(xVehicle,u)' + J2(xVehicle,u)* UEst * J2(xVehicle,u)';
    PPredvm = J1(xVehicle,u)*PEst(1:3,4:end);
    PPredmm = PEst(4:end,4:end);
    
    % Aggregate robot and landmark poses.
    xPred = [xVehiclePred;xMap];
    PPred = [PPredvv PPredvm;
        PPredvm' PPredmm];
    
    ObsNoise = [0; 0]; %randn(2,1);
    [z,iFeature] = GetObservation(ObsNoise,RTrue);
    
    
    if(~isempty(z)) % Valid observation received.
        % Have we seen this feature before?
        if( ~isnan(MappedLandFeatures(iFeature,1)))
            
            % Predict the observation.
            FeatureIndex = MappedLandFeatures(iFeature,1);
            xFeature = xPred(FeatureIndex:FeatureIndex+1);
            zPred = DoObservationModel(xVehicle,xFeature);
            [jHxv,jHxf] = GetObsJacs(xVehicle,xFeature);
            jH = zeros(2,length(xEst));
            jH(:,FeatureIndex:FeatureIndex+1) = jHxf;
            jH(:,1:3) = jHxv;
            
            % Kalman update.
            Innov = z-zPred;
            Innov(2) = AngleWrapping(Innov(2));
            
            S = jH*PPred*jH'+REst;
            W = PPred*jH'*inv(S);
            xEst = xPred+ W*Innov;
            
            PEst = PPred-W*S*W';
            
            PEst = 0.5*(PEst+PEst');
        % New feature - add to state vector estimate.
        else
            
            nStates = length(xEst);
            xFeature = xVehicle(1:2)+ [z(1)*cos(z(2)+xVehicle(3));z(1)*sin(z(2)+xVehicle(3))];
            xEst = [xEst;xFeature];
            [jGxv, jGz] = GetNewFeatureJacs(xVehicle,z);
            
            % M=| 1 0 0  0 0
            %   | 0 1 0  0 0
            %   | 0 0 1  0 0
            %   -------------
            %   |
            
            M = [eye(nStates), zeros(nStates,2);
                jGxv zeros(2,nStates-3)  , jGz];
            
            PEst = M*blkdiag(PEst,REst)*M';
            
            % Store ID of new landmark.
            MappedLandFeatures(iFeature,:) = [length(xEst)-1, length(xEst)];
            
        end
     
    % No observation received.
    else
        xEst = xPred;
        PESt = PPred;
    end
    
    %% Mapping
    %     % Take measurement - discrete.
    %     ind = knnsearch(X_train, xEst(1:2)');
    %     X_test = [X_test; X_train(ind, :)];
    %     Y_test = [Y_test; Y_train(ind)];
    %
    % Take measurement - continuous.
    X_test = [X_test; xEst(1:2)'];
    X_test_gt = [X_test_gt; xVehicleTrue(1:2)'];
    Y_test = [Y_test; interp2(reshape(X_train(:,1),30,30), ...
        reshape(X_train(:,2),30,30), ...
        reshape(Y_train,30,30), xEst(1), xEst(2))];
    
    % Do GP regression.
    [ymu, ys, fmu, fs, ~ , post] = ...
        gp(hyp_trained, inf_func, [], cov_func, lik_func, ...
        X_test, Y_test, X_predict);
    
    %% Visualization
    a = axis;
    clf;
    axis(a);hold on;
    grid minor
    
    scatter(X_predict(:,1), X_predict(:,2), 100, ymu, 'filled');
    caxis([0 50])
    
    n  = length(xEst);
    nF = (n-3)/2;
    
    DoVehicleGraphics(xEst(1:3),PEst(1:3,1:3),3,[0 1]);
    %disp(xEst(1:3))
    %disp(PEst(1:3,1:3))
    
    % Plot lines to observed landmarks.
    if(~isnan(z))
        h = line([xEst(1),xFeature(1)],[xEst(2),xFeature(2)]);
        set(h,'linestyle',':');
        
    end
    % Plot the covariance ellipse of each landmark state.
    for (i = 1:nF)
        iF = 3+2*i-1;
        plot(xEst(iF),xEst(iF+1),'kd', 'MarkerFaceColor', 'g', 'MarkerSize', 14 );
        PlotEllipse(xEst(iF:iF+1),PEst(iF:iF+1,iF:iF+1),3);
    end
    
    drawnow;
    
    if (mod(k,200) == 0)
        colorbar
        keyboard
    end
    
end












