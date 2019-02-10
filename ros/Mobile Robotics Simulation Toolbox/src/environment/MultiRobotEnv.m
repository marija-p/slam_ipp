classdef MultiRobotEnv < matlab.System
    % MULTIROBOTENV 2D Multi-Robot Environment
    %
    % Displays the pose (position and orientation) of multiple objects in a 
    % 2D environment. Additionally has the option to display a map as a 
    % robotics.OccupancyGrid or robotics.BinaryOccupancyGrid, object
    % trajectories, waypoints, lidar scans, and/or objects.
    %
    % For more information, see <a href="matlab:edit mrsDocMultiRobotEnv">the documentation page</a>
    %
    % Copyright 2018 The MathWorks, Inc.

    %% PROPERTIES
    % Public (user-visible) properties
    properties(Nontunable)
        numRobots = 1;              % Number of robots
        robotRadius = 0;            % Robot radii [m]
        showTrajectory = false;     % Show trajectory
        mapName = '';               % Map
        hasWaypoints = false;       % Accept waypoints
        hasLidar = false;           % Accept lidar inputs
        hasObjDetector = false;     % Accept object detections
        hasRobotDetector = false;   % Accept robot detections
        plotSensorLines = true;     % Plot sensor lines
        showRobotIds = true;        % Show robot IDs
    end
    properties
       % Lidar
       sensorOffset = {[0 0]};          % Lidar sensor offset (x,y) [m] 
       scanAngles = {[-pi/4,0,pi/4]};   % Scan angles [rad]
       % Object detectors
       objDetectorOffset = {[0 0]};     % Object detector offset (x,y) [m] 
       objDetectorAngle = 0;            % Object detector angle [rad]
       objDetectorFOV = pi/4;           % Object detector field of view [rad] 
       objDetectorMaxRange = 5;         % Object detector maximum range [m]
       objectColors = [1 0 0;0 1 0;0 0 1]; % Object label colors [RGB rows]
       % Robot detectors
       robotDetectorOffset = {[0 0]};   % Robot detector offset (x,y) [m] 
       robotDetectorAngle = 0;          % Robot detector angle [rad]
       robotDetectorFOV = pi/4;         % Robot detector field of view [rad] 
       robotDetectorMaxRange = 5;       % Robot detector maximum range [m]
       % Other
       Poses; % Robot poses (x,y,theta) [m,m,rad] 
    end

    % Private properties
    properties(Access = private)
        map;                        % Occupancy grid representing the map
        fig;                        % Figure window
        ax;                         % Axes for plotting
        RobotHandle;                % Handle to robot body marker or circle
        OrientationHandle;          % Handle to robot orientation line
        LidarHandles;               % Handle array to lidar lines
        TrajHandle;                 % Handle to trajectory plot
        trajX;                      % X Trajectory points
        trajY;                      % Y Trajectory points
        WaypointHandle;             % Handle to waypoints
        ObjectHandle;               % Handle to objects
        ObjDetectorHandles = {};    % Handle array to object detector lines
        RobotDetectorHandles = {};  % Handle array to robot detector lines
        IdHandles;                  % Handle array to robot IDs
    end

    %% METHODS
    methods    
        % Constructor: Takes number of robots as mandatory argument
        function obj = MultiRobotEnv(N)
            obj.numRobots = N;
        end        
    end
        
    methods(Access = protected)
        
        % Setup method: Initializes all necessary graphics objects
        function setupImpl(obj)                 
            % Setup the visualization
            setupVisualization(obj);                
        end

        % Step method: Updates visualization based on inputs
        function stepImpl(obj,robotIndices,poses,varargin)

            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end            
            
            % Unpack the optional arguments
            idx = 1;
            % Waypoints
            if obj.hasWaypoints
                waypoints = varargin{idx};
                idx = idx + 1;
            else
                waypoints = [];
            end
            % Lidar ranges
            if any(obj.hasLidar) 
                ranges = varargin{idx};
                idx = idx + 1;
            else
                ranges = cell(1,obj.numRobots);
            end
            % Objects
            if any(obj.hasObjDetector) 
                objects = varargin{idx};
            else
                objects = [];
            end
            
            % Draw the waypoints and objects
            drawWaypointsAndObjects(obj,waypoints,objects);
           
            % Draw the robots
            if ~isempty(robotIndices)
                drawRobots(obj,robotIndices,poses,ranges);
            end
            
            % Update the figure
            drawnow('limitrate')           
        end    
        
    end
    
    methods (Access = public) 
        
        % Performs all the visualization setup. It is separate from the
        % setup method, since it can be called independently as well.
        function setupVisualization(obj)
            % Convert scalar flags to arrays based on number of robots
            if numel(obj.robotRadius) ~= obj.numRobots
                obj.robotRadius = repmat(obj.robotRadius,[1,obj.numRobots]);
            end
            if numel(obj.showTrajectory) ~= obj.numRobots
                obj.showTrajectory = repmat(obj.showTrajectory,[1,obj.numRobots]);
            end
            if numel(obj.hasLidar) ~= obj.numRobots
                obj.hasLidar = repmat(obj.hasLidar,[1,obj.numRobots]);
            end
            if numel(obj.hasObjDetector) ~= obj.numRobots
                obj.hasObjDetector = repmat(obj.hasObjDetector,[1,obj.numRobots]);
            end
            if numel(obj.hasRobotDetector) ~= obj.numRobots
                obj.hasRobotDetector = repmat(obj.hasRobotDetector,[1,obj.numRobots]);
            end
            
            % Initialize poses
            obj.Poses = nan(3,obj.numRobots);
            
            % Create figure
            FigureName = 'Multi-Robot Environment';
            FigureTag = 'MultiRobotEnvironment';
            existingFigures = findobj('type','figure','tag',FigureTag);
            if ~isempty(existingFigures)
                obj.fig = figure(existingFigures(1)); % bring figure to the front
                clf;
            else
                obj.fig = figure('Name',FigureName,'tag',FigureTag);
            end
            
            % Create global axes
            obj.ax = axes('parent',obj.fig);
            hold(obj.ax,'on');
            
            % Show the map
            obj.map = internal.createMapFromName(obj.mapName);
            if ~isempty(obj.map)
                show(obj.map,'Parent',obj.ax);
            end
            
            % Initialize robot plot
            obj.OrientationHandle = cell(obj.numRobots,1);
            obj.RobotHandle = cell(obj.numRobots,1);
            for rIdx = 1:obj.numRobots
                obj.OrientationHandle{rIdx} = plot(obj.ax,0,0,'r','LineWidth',1.5);
                if obj.robotRadius(rIdx) > 0
                    % Finite size robot
                    [x,y] = internal.circlePoints(0,0,obj.robotRadius(rIdx),17);
                    obj.RobotHandle{rIdx} = plot(obj.ax,x,y,'b','LineWidth',1.5);
                else
                    % Point robot
                    obj.RobotHandle{rIdx} = plot(obj.ax,0,0,'bo', ...
                         'LineWidth',1.5,'MarkerFaceColor',[1 1 1]);
                end
                % Initialize robot IDs, if enabled
                if obj.showRobotIds
                    obj.IdHandles{rIdx} = text(0,0,num2str(rIdx), ... 
                               'Color','b','FontWeight','bold');
                end
            end
            
            % Initialize trajectory
            obj.TrajHandle = cell(obj.numRobots,1);
            obj.trajX = cell(obj.numRobots,1);
            obj.trajY = cell(obj.numRobots,1);
            for rIdx = 1:obj.numRobots
                if obj.showTrajectory(rIdx)
                    obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'b.-');
                    obj.trajX{rIdx} = [];
                    obj.trajY{rIdx} = [];
                end
            end
            
            % Initialize waypoints
            if obj.hasWaypoints
                obj.WaypointHandle = plot(obj.ax,0,0, ...
                    'rx','MarkerSize',10,'LineWidth',2);
            end
            
            % Initialize lidar lines
            obj.LidarHandles = cell(obj.numRobots,1);
            for rIdx = 1:obj.numRobots
                if obj.hasLidar(rIdx)
                    for idx = 1:numel(obj.scanAngles{rIdx})
                        obj.LidarHandles{rIdx}(idx) = plot(obj.ax,0,0,'b--');
                    end
                end
            end
            
            % Initialize objects and object detector lines
            obj.ObjectHandle = scatter(obj.ax,[],[],75,'s','filled', ...
                    'LineWidth',2);
            obj.ObjDetectorHandles = cell(obj.numRobots,1);
            for rIdx = 1:obj.numRobots
                if obj.hasObjDetector(rIdx)
                    objDetectorFormat = 'g-.';
                    obj.ObjDetectorHandles{rIdx}(1) = plot(obj.ax,0,0,objDetectorFormat); % Left line
                    obj.ObjDetectorHandles{rIdx}(2) = plot(obj.ax,0,0,objDetectorFormat); % Right line
                end
            end

            % Initialize robot detector lines
            obj.RobotDetectorHandles = cell(obj.numRobots,1);
            for rIdx = 1:obj.numRobots
                    robotDetectorFormat = 'm:';
                    obj.RobotDetectorHandles{rIdx} = [ plot(obj.ax,0,0,robotDetectorFormat,'LineWidth',1.5), ... % Left line
                                                       plot(obj.ax,0,0,robotDetectorFormat,'LineWidth',1.5) ];    % Right line
            end
            
            % Final setup
            title(obj.ax,'Multi-Robot Visualization');
            hold(obj.ax,'off');
            axis equal
            
        end
        
        % Helper method to draw the waypoints and objects,
        % which are independent of the robot
        function drawWaypointsAndObjects(obj,waypoints,objects)
            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end
            
            % Update waypoints
            if obj.hasWaypoints && (numel(waypoints) > 1)
                set(obj.WaypointHandle,'xdata',waypoints(:,1), ...
                                       'ydata',waypoints(:,2));
            else
                set(obj.WaypointHandle,'xdata',[], ...
                                       'ydata',[]);
            end
            
            % Update the objects
            if numel(objects) <= 1
                set(obj.ObjectHandle,'xdata',[],'ydata',[],'cdata',[]);
            else
                % Plot the objects with their corresponding colors
                if size(obj.objectColors,1) == 1
                    colorData = obj.objectColors; % Use the single color specified
                else
                    colorData = obj.objectColors(objects(:,3),:); % Use the color based on labels
                end
                set(obj.ObjectHandle,'xdata',objects(:,1),...
                    'ydata',objects(:,2),'cdata',colorData);
            end
        end
        
        % Helper method to draw all robots (calls drawRobot)
        function drawRobots(obj,robotIndices,poses,ranges)
            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end
            
            % Draw each robot and its sensors              
            % Single-robot case
            if numel(robotIndices) == 1
               obj.Poses(:,robotIndices) = poses;
               drawRobot(obj,robotIndices,poses,ranges); 
            % Multi-robot case
            else
                for rIdx = robotIndices
                    pose = poses(:,rIdx);
                    obj.Poses(:,rIdx) = pose;
                    drawRobot(obj,rIdx,pose,ranges{rIdx}); 
                end
            end
        end
        
        % Helper method to draw the robot and its sensors at each step
        function drawRobot(obj,rIdx,pose,ranges)
            % Unpack the pose input into (x, y, theta)
            x = pose(1);
            y = pose(2);
            theta = pose(3);
            
            % Update the trajectory
            if obj.showTrajectory(rIdx)
                obj.trajX{rIdx}(end+1) = x;
                obj.trajY{rIdx}(end+1) = y;
                set(obj.TrajHandle{rIdx},'xdata',obj.trajX{rIdx}, ...
                    'ydata',obj.trajY{rIdx});
            end
            
            % Update the robot pose
            xAxesLim = get(obj.ax,'XLim');
            lineLength = diff(xAxesLim)/20;
            r = obj.robotRadius(rIdx);
            if r > 0
                % Finite radius case
                [xc,yc] = internal.circlePoints(x,y,r,17);
                set(obj.RobotHandle{rIdx},'xdata',xc,'ydata',yc);
                len = max(lineLength,2*r); % Plot orientation based on radius unless it's too small
                xp = [x, x+(len*cos(theta))];
                yp = [y, y+(len*sin(theta))];
                set(obj.OrientationHandle{rIdx},'xdata',xp,'ydata',yp);
            else
                % Point robot case
                xp = [x, x+(lineLength*cos(theta))];
                yp = [y, y+(lineLength*sin(theta))];
                set(obj.RobotHandle{rIdx},'xdata',x,'ydata',y);
                set(obj.OrientationHandle{rIdx},'xdata',xp,'ydata',yp);
            end
            % Show robot IDs, if enabled
            if obj.showRobotIds
                set(obj.IdHandles{rIdx},'Position',[x y] - max(1.25*r,lineLength*0.5));
            end
            
            % Update lidar lines
            if obj.hasLidar(rIdx) && obj.plotSensorLines
                
                % Find the sensor location(s)
                offsetVec = [cos(theta) -sin(theta); ...
                             sin(theta)  cos(theta)]*obj.sensorOffset{rIdx}';
                sensorLoc = [x,y] + offsetVec';
                
                scanAngs = obj.scanAngles{rIdx};
                for idx = 1:numel(ranges)
                    if ~isnan(ranges(idx))
                        
                        % If there is a single sensor offset, use that value
                        if size(sensorLoc,1) == 1
                            sensorPos = sensorLoc;
                            % Else, use the specific index's sensor location
                        else
                            sensorPos = sensorLoc(idx,:);
                        end
                        
                        alpha = theta + scanAngs(idx);
                        lidarX = sensorPos(1) + [0, ranges(idx)*cos(alpha)];
                        lidarY = sensorPos(2) + [0, ranges(idx)*sin(alpha)];
                        set(obj.LidarHandles{rIdx}(idx),'xdata',lidarX, ...
                            'ydata',lidarY);
                    else
                        set(obj.LidarHandles{rIdx}(idx),'xdata',[],'ydata',[]);
                    end
                end
            end
            
            % Update object and object detector lines
            if obj.hasObjDetector(rIdx) && obj.plotSensorLines
                
                % Find the sensor location(s)
                offsetVec = [cos(theta) -sin(theta); ...
                             sin(theta)  cos(theta)]*obj.objDetectorOffset{rIdx}';
                sensorLoc = [x,y] + offsetVec';
                
                % Plot the object detector lines
                maxRange = obj.objDetectorMaxRange(rIdx);
                % Left line
                alphaLeft = theta + obj.objDetectorAngle(rIdx) + obj.objDetectorFOV(rIdx)/2;
                if ~isempty(obj.map)
                    intPtsLeft = rayIntersection(obj.map,[sensorLoc alphaLeft],0,maxRange);
                else
                    intPtsLeft = NaN;
                end
                if ~isnan(intPtsLeft)
                    objLeftX = [sensorLoc(1) intPtsLeft(1)];
                    objLeftY = [sensorLoc(2) intPtsLeft(2)];
                else
                    objLeftX = sensorLoc(1) + [0, maxRange*cos(alphaLeft)];
                    objLeftY = sensorLoc(2) + [0, maxRange*sin(alphaLeft)];
                end
                set(obj.ObjDetectorHandles{rIdx}(1), ...
                    'xdata',objLeftX,'ydata',objLeftY);
                % Right line
                alphaRight = theta + obj.objDetectorAngle(rIdx) - obj.objDetectorFOV(rIdx)/2;
                if ~isempty(obj.map)
                    intPtsRight = rayIntersection(obj.map,[sensorLoc alphaRight],0,maxRange);
                else
                    intPtsRight = NaN;
                end
                if ~isnan(intPtsRight)
                    objRightX = [sensorLoc(1) intPtsRight(1)];
                    objRightY = [sensorLoc(2) intPtsRight(2)];
                else
                    objRightX = sensorLoc(1) + [0, maxRange*cos(alphaRight)];
                    objRightY = sensorLoc(2) + [0, maxRange*sin(alphaRight)];
                end
                set(obj.ObjDetectorHandles{rIdx}(2), ...
                    'xdata',objRightX,'ydata',objRightY);
            end
            
            % Update robot detector lines
            if obj.hasRobotDetector(rIdx) && obj.plotSensorLines
                
                % Find the sensor location(s)
                offsetVec = [cos(theta) -sin(theta); ...
                             sin(theta)  cos(theta)]*obj.robotDetectorOffset{rIdx}';
                sensorLoc = [x,y] + offsetVec';
                
                % Plot the robot detector lines
                maxRange = obj.robotDetectorMaxRange(rIdx);
                % Left line
                alphaLeft = theta + obj.robotDetectorAngle(rIdx) + obj.robotDetectorFOV(rIdx)/2;
                if ~isempty(obj.map)
                    intPtsLeft = rayIntersection(obj.map,[sensorLoc alphaLeft],0,maxRange);
                else
                    intPtsLeft = NaN;
                end
                if ~isnan(intPtsLeft)
                    objLeftX = [sensorLoc(1) intPtsLeft(1)];
                    objLeftY = [sensorLoc(2) intPtsLeft(2)];
                else
                    objLeftX = sensorLoc(1) + [0, maxRange*cos(alphaLeft)];
                    objLeftY = sensorLoc(2) + [0, maxRange*sin(alphaLeft)];
                end
                set(obj.RobotDetectorHandles{rIdx}(1), ...
                    'xdata',objLeftX,'ydata',objLeftY);
                % Right line
                alphaRight = theta + obj.robotDetectorAngle(rIdx) - obj.robotDetectorFOV(rIdx)/2;
                if ~isempty(obj.map)
                    intPtsRight = rayIntersection(obj.map,[sensorLoc alphaRight],0,maxRange);
                else
                    intPtsRight = NaN;
                end
                if ~isnan(intPtsRight)
                    objRightX = [sensorLoc(1) intPtsRight(1)];
                    objRightY = [sensorLoc(2) intPtsRight(2)];
                else
                    objRightX = sensorLoc(1) + [0, maxRange*cos(alphaRight)];
                    objRightY = sensorLoc(2) + [0, maxRange*sin(alphaRight)];
                end
                set(obj.RobotDetectorHandles{rIdx}(2), ...
                    'xdata',objRightX,'ydata',objRightY);
            end            
            
            
        end    
           
        % Attaches all properties associated with a LidarSensor object
        function attachLidarSensor(obj,rIdx,lidar)
            if numel(obj.hasLidar) ~= obj.numRobots
                obj.hasLidar = repmat(obj.hasLidar,[1,obj.numRobots]);
            end
            obj.hasLidar(rIdx) = true;
            obj.sensorOffset{rIdx} = lidar.sensorOffset;
            obj.scanAngles{rIdx} = lidar.scanAngles;
            
            % Ensure to use the same map as the visualizer
            release(lidar);
            lidar.mapName = obj.mapName;
            lidar.isMultiRobot = true;
            lidar.robotIdx = rIdx;
            setEnvironment(lidar,obj);
        end
        
        % Attaches all properties associated with an ObjectDetector object
        function attachObjectDetector(obj,rIdx,detector)
            if numel(obj.hasObjDetector) ~= obj.numRobots
                obj.hasObjDetector = repmat(obj.hasObjDetector,[1,obj.numRobots]);
            end 
            obj.hasObjDetector(rIdx) = true;
            obj.objDetectorOffset{rIdx} = detector.sensorOffset;
            obj.objDetectorAngle(rIdx) = detector.sensorAngle;
            obj.objDetectorFOV(rIdx) = detector.fieldOfView;
            obj.objDetectorMaxRange(rIdx) = detector.maxRange;
            
            % Ensure to use the same map as the visualizer
            release(detector);
            detector.mapName = obj.mapName;
        end

        % Attaches all properties associated with a RobotDetector object
        function attachRobotDetector(obj,detector)
            release(obj);
            
            if numel(obj.hasRobotDetector) ~= obj.numRobots
                obj.hasRobotDetector = repmat(obj.hasRobotDetector,[1,obj.numRobots]);
            end
            
            rIdx = detector.robotIdx;
            obj.hasRobotDetector(rIdx) = true;
            obj.robotDetectorOffset{rIdx} = detector.sensorOffset;
            obj.robotDetectorAngle(rIdx) = detector.sensorAngle;
            obj.robotDetectorFOV(rIdx) = detector.fieldOfView;
            obj.robotDetectorMaxRange(rIdx) = detector.maxRange;
        end
        
    end
        
end