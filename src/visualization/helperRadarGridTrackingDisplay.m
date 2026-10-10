classdef helperRadarGridTrackingDisplay < matlab.System
    % This is a helper class and may be removed or modified in a future
    % release.
    
    % This class provides the visualizations for the radar grid-based
    % tracking example.
    
    % Handles to panel plots
    properties
        FrontView
        GridView
        SensorViews
    end
        
    properties
        Figure
    end
    
    properties
        DynamicMapAxes
    end
    
    properties
        Record = false;
    end

    properties (Access = protected)
        Snapshots
        pFrames
        DownsampleFactor = 3;
    end
    
    methods
        function obj = helperRadarGridTrackingDisplay(varargin)
            setProperties(obj,nargin,varargin{:});
        end
        
        function snapnow(obj)
            if isPublishing(obj)
                obj.Snapshots{end+1} = getframe(obj.Figure);
            end
        end
        
        function showSnaps(obj, idx)
            if nargin == 1
                idx = 1:numel(obj.Snapshots);
            end
            allSnaps = obj.Snapshots(idx);
            n = numel(allSnaps);
            p = floor(sqrt(n));
            q = floor(n/p);
            diff = n - p*q;
            while diff > 0
                q = q + 1;
                diff = n - p*q;
            end
            for i = 1:n
                ax = subplot(p,q,i);
                imagesc(ax,allSnaps{i}.cdata);
            end
        end
    end
    
    methods
        function writeAnimation(obj, fName, delay)
            if nargin == 2
                delay = 0;
            end
            obj.DownsampleFactor = 2;
            if obj.Record
                frames = obj.pFrames;
                imSize = size(frames{1}.cdata);
                im = zeros(imSize(1),imSize(2),1,floor(numel(frames)/obj.DownsampleFactor),'uint8');
                map = [];
                count = 1;
                for i = 1:obj.DownsampleFactor:numel(frames)
                    if isempty(map)
                        [im(:,:,1,count),map] = rgb2ind(frames{i}.cdata,256,'nodither');
                    else
                        im(:,:,1,count) = rgb2ind(frames{i}.cdata,map,'nodither');
                    end
                    count = count + 1;
                end
                imwrite(im,map,[fName,'.gif'],'DelayTime',delay,'LoopCount',inf);
            end
        end
        
        function writeMovie(obj, fName, frameRate)
            vidWriter = VideoWriter([fName,'.avi']);
            vidWriter.FrameRate = frameRate;
            vidWriter.open();
            for i = 1:numel(obj.pFrames)
                vidWriter.writeVideo(obj.pFrames{i});
            end
            vidWriter.close();
        end
    end
    
    
    methods (Access = protected)
        function setupImpl(obj, varargin)
            obj.Figure = figure('Units','normalized','Position',[0 0 1 0.8]);
            
            if obj.Record
                obj.Figure.Position = [0 0 0.6 0.48];
            end

            if isPublishing(obj)
                obj.Figure.Visible = 'off';
            end
            
            obj.FrontView = createFrontView(obj,obj.Figure);
            
            obj.SensorViews = createSensorViews(obj,obj.Figure,varargin{3});
            
            obj.GridView = createGridView(obj,obj.Figure);
            
            scenario = varargin{1};
            
            updatePlotColors(obj, scenario);
        end
        
        function gridView = createGridView(obj,fig)
            p4 = uipanel(fig,'Units','normalized','Position',[0.5 0 0.5 1],...
                'Title','Grid-based Tracker');
            
            snapInfo = struct('TrackID',1,...
                'SnapTimes',1:0.5:5,...
                'Snapshots',{{}});
            
            snapInfo(2) = snapInfo(1);
            snapInfo(2).TrackID = 6;
            snapInfo(2).SnapTimes = 3:0.5:7;
            
            snapInfo(3) = snapInfo(1);
            snapInfo(3).TrackID = 20;
            snapInfo(3).SnapTimes = 10:0.5:14;
            
            snapInfo(4) = snapInfo(1);
            snapInfo(4).TrackID = [3 15];
            snapInfo(4).SnapTimes = 5:0.5:9;
            
            snapInfo(5) = snapInfo(1);
            snapInfo(5).TrackID = 18;
            snapInfo(5).SnapTimes = 8:0.5:12;

            gridView = helperRadarGridTrackingDisplayPanel('Parent',p4,...
                'ViewLocation',[0 0 160],...
                'ViewAngles',[0 90 0],...
                'PlotTruth',false,...
                'PlotRoads',false,...
                'PlotCoverage',false,...
                'DarkMode',true,...
                'PanWithEgo',false,...
                'PlotPointClouds',false,...
                'PlotCoverage',true,...
                'ReferenceEgo',true);
            
            gridView.SnapInformation = snapInfo;
        end
        
        function frontView = createFrontView(obj, fig)
            p1 = uipanel(fig,'Units','normalized','Position',[0 0.5 0.5 0.5],...
                'Title','Ground truth - Front View');
            
            frontView = helperRadarGridTrackingDisplayPanel('Parent',p1,...
                'ViewLocation',[-11.75 0 2.1],...
                'ViewAngles',[0 0 0],...
                'PlotCoverage',false,...
                'PlotPointClouds',true,...
                'PlotTracks',false,...
                'ShowLegend',false);
        end
        
        function sensorViews = createSensorViews(obj, fig, radars)
            order = [3 2 4 1 6 5];
            radars = radars([3 2 4 1 6 5]);
            str = ["Left Rear Radar","Left Front Radar","Rear Radar","Front Radar","Right Rear Radar","Right Front Radar"];
            xpositions = linspace(0,0.5,4);
            ypositions = linspace(0,0.5,3);
            dx = diff(xpositions);
            dy = diff(ypositions);
            dx = dx(1);
            dy = dy(1);
            
            idx = 1;
            for i = 1:3
                for k = 1:2
                    p = uipanel(fig,'Units','normalized','Position',[xpositions(i) ypositions(k) dx dy],...
                        'Title',str(idx));
                    sensorToPlot = false(numel(radars),1);
                    sensorToPlot(order(idx)) = true;
                    loc = radars{idx}.MountingLocation(:);
                    angles = radars{idx}.MountingAngles(:)';
                    move = [-11.75 0 2.1];
                    q = quaternion(angles,'eulerd','ZYX','frame');
                    r = rotatepoint(q,move);
                    loc = loc + r(:);
                    
                    sensorViews{idx} =  helperRadarGridTrackingDisplayPanel('Parent',p,...
                        'ViewLocation',loc,...
                        'ViewAngles',angles,...
                        'PlotCoverage',false,...
                        'PlotTruth',false,...
                        'PlotRoads',false,...
                        'PlotPointClouds',true,...
                        'PlotTracks',false,...
                        'ShowLegend',false,...
                        'DarkMode',true,...
                        'SensorToPlot',sensorToPlot);
                    idx = idx + 1;
                end
            end
        end
        function updatePlotColors(obj, scenario)
            for i = 2:numel(scenario.Actors)
                a = scenario.Actors(i);
                v = a.Velocity;
                if norm(v) <= 0
                    rgb = 0.4*[1 1 1];
                    a.PlotColor(:) = rgb;
                end
            end
        end
        
        function stepImpl(obj, varargin)
            obj.FrontView(varargin{:});
            obj.GridView(varargin{:});
            for i = 1:numel(obj.SensorViews)
                this = obj.SensorViews{i};
                this(varargin{:});
            end
            
            tracker = varargin{end-1};
            showDynamicMap(tracker,'InvertColors',true,'Parent',obj.GridView.Axes);
            if isempty(obj.DynamicMapAxes)
                obj.DynamicMapAxes = obj.GridView.Axes;
                view(obj.DynamicMapAxes,-90,90);
                zlim(obj.DynamicMapAxes,[-100 100]);
            end
            
            tracks = varargin{end};
            ego = varargin{2};
            takeSnapshot(obj.GridView, tracks, ego);
            
            if obj.Record
                obj.pFrames{end+1} = getframe(obj.Figure);
            end
            drawnow;
        end
    end
    
    methods
        function writeScenarioAnimation(obj, scenario, egoVehicle, radars)
            f = figure('Units','normalized','Position',[0 0 0.6 0.6]);
           
            frontView = createFrontView(obj,f);
            frontView.Parent.Position = [0 0.5 1 0.5];
                
            sensorViews = createSensorViews(obj, f, radars);
            for i = 1:numel(sensorViews)
                sensorViews{i}.Parent.Position(1) = 2*sensorViews{i}.Parent.Position(1);
                sensorViews{i}.Parent.Position(3) = 2*sensorViews{i}.Parent.Position(3);
            end
            
            obj.Record = true;
            obj.DownsampleFactor = 3;
            
            restart(scenario);
            for i = 1:numel(radars)
                release(radars{i});
            end
            
            % Load data
            load('MultiRadarGridBasedTrackingData','radarPointCloudLog','radarSpecification');
            count = 1;
            while advance(scenario)
                frontView(scenario,egoVehicle,radarSpecification,radarPointCloudLog{count});
                for i = 1:numel(sensorViews)
                    sensorViews{i}(scenario,egoVehicle,radars,radarPointCloudLog{count});
                end
                count = count + 1;
                obj.pFrames{end+1} = getframe(f);
            end
            
            writeAnimation(obj,'multiRadarTrackingScenario',0.1);
        end
    end
    
    methods     
        function tf = isPublishing(~)
            tf = false;
            try 
                s = numel(dbstack);
                tf = s > 5;
            catch
            end
            % tf = true;
        end
    end
    
end