classdef helperRadarGridTrackingDisplayPanel < matlab.System
    % This is a helper class and may be removed or modified in a future
    % release. 
    
    % This class provides the display for a single panel of the radar
    % grid-based tracking example
    
    
    properties
        TrackPlotterPatch
        Resolution
    end
    
    properties
        SnapInformation
        Snapshots
    end
    
    properties
        ScenarioPlotter
        TrackPlotter
        CoveragePlotter
        PointCloudPlotter
        DynamicMapAxes
        ViewAngles = [0 90 0];
        ViewLocation = [0 0 30];
        PanWithEgo = true;
        ReferenceEgo = false;
    end
    
    properties
        ColorOrder;
        GroundTruthColorIndex = 0;
        TrackColorIndex = 2;
        CoverageColorIndex = 2;
    end
    
    properties
        Parent
        Axes
        DarkMode = false;
    end
    
    properties
        SensorToPlot = true(6,1);
        ShowLegend = false;
    end
    
    properties
        PlotPointClouds = true;
        PlotCoverage = true;
        PlotTracks = true;
        PlotTruth = true;
        PlotRoads = true;
    end
    
    methods
        function obj = helperRadarGridTrackingDisplayPanel(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods
        function showSnapshots(obj, trkID)
            for i = 1:numel(obj.SnapInformation)
                if any(obj.SnapInformation(i).TrackID == trkID)
                    thisSnapInfo = obj.SnapInformation(i);
                    break;
                end
            end
            if ~isempty(thisSnapInfo)
                allSnaps = thisSnapInfo.Snapshots;
                allSnapTimes = thisSnapInfo.SnapTimes;
                if ~isempty(allSnaps)
                    figure('Units','normalized','Position',[0 0 1 1]);
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
                        R = allSnaps{i}.cdata(:,:,1);
                        G = allSnaps{i}.cdata(:,:,2);
                        B = allSnaps{i}.cdata(:,:,3);
                        R = fliplr(R');
                        G = fliplr(G');
                        B = fliplr(B');
                        img = cat(3,R,G,B);
                        im = imagesc(ax, img);
                        xLimits = allSnaps{i}.XLimits;
                        yLimits = allSnaps{i}.YLimits;
                        topLeft = [xLimits(1) yLimits(1)] + 1/(2*obj.Resolution);
                        bottomRight = [xLimits(2) yLimits(2)] - 1/(2*obj.Resolution);
                        im.XData = [topLeft(1) bottomRight(1)];
                        im.YData = [bottomRight(2) topLeft(2)];
                        ax.XLim = xLimits;
                        ax.YLim = yLimits;
                        ax.YDir = 'normal';
                        view(ax,-90,90);
                        ax.Title.String = ['TrackID = ',num2str(trkID),' Time = ',num2str(allSnapTimes(i))];
                    end
                end
            end
        end
        
        function takeSnapshot(obj, tracks, ego)
            if obj.ReferenceEgo
                tracks = transformToEgo(tracks,ego);
            end
            
            if isPublishing(obj) && ~isempty(tracks)
                obj.TrackPlotterPatch.LineWidth = 5;
                h = findall(obj.Axes,'Tag','tpTrackLabels_Tracks');
                for i = 1:numel(h)
                    h(i).FontSize = 35;
                    h(i).FontWeight = 'bold';
                end
                xlim = obj.Axes.XLim;
                ylim = obj.Axes.YLim;
                zlim = obj.Axes.ZLim;
                ids = [tracks.TrackID];
                [pos,~,dim] = parseTracks(tracks);
                currentTime = tracks(1).UpdateTime;
                snapInformation = obj.SnapInformation;
                for i = 1:numel(snapInformation)
                    trkID = snapInformation(i).TrackID;
                    snapTimes = snapInformation(i).SnapTimes;
                    if any(abs(snapTimes - currentTime) < 1e-6)
                        obj.TrackPlotter;
                        xLimTracks = nan(numel(trkID),2);
                        yLimTracks = nan(numel(trkID),2);
                        for m = 1:numel(trkID)
                            thisPos = pos(ids == trkID(m),:);
                            thisDim = dim(ids == trkID(m));
                            if ~any(ids == trkID(m))
                                continue;
                            end
                            L = thisDim.Length;
                            W = thisDim.Width;
                            span = max(4,min(2*max(L,W),8));
                            xLimTracks(m,:) = thisPos(1) + [-span span];
                            yLimTracks(m,:) = thisPos(2) + [-span span];
                        end
                        xLimAxes = [min(xLimTracks(:,1)) max(xLimTracks(:,2))];
                        yLimAxes = [min(yLimTracks(:,1)) max(yLimTracks(:,2))];
                        if any(isnan(xLimAxes)) || any(isnan(yLimAxes))
                            continue;
                        end
                        obj.Axes.XLim = xLimAxes;
                        obj.Axes.YLim = yLimAxes;
                        obj.Axes.ZLim = [-100 100];
                        snapData = getframe(obj.Axes);
                        snapData.XLimits = xLimAxes;
                        snapData.YLimits = yLimAxes;
                        obj.SnapInformation(i).Snapshots{end+1} = snapData;
                        obj.Axes.XLim = xlim;
                        obj.Axes.YLim = ylim;
                        obj.Axes.ZLim = zlim;
                    end
                end
                for i = 1:numel(h)
                    h(i).FontSize = 10;
                    h(i).FontWeight = 'normal';
                end
            end
            obj.TrackPlotterPatch.LineWidth = 2;
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj, scenario, egoVehicle, radars, ptCloud, tracker, tracks)
            if obj.DarkMode
                obj.ColorOrder = darkColorOrder;
            else
                obj.ColorOrder = standardColorOrder;
            end
            
            if nargin > 5
                obj.Resolution = tracker.GridResolution;
            end
            
            if isempty(obj.Parent)
                fig = figure('Units','normalized','Position',[0 0 0.9 0.9]);
                obj.Parent = fig;
            end
            
            ax = axes(obj.Parent);
            
            if obj.DarkMode  %Dark-mode
                if isa(obj.Parent,'matlab.ui.Figure')
                    obj.Parent.Color = 0.157*[1 1 1];
                elseif isa(obj.Parent,'matlab.ui.container.Panel')
                    obj.Parent.BackgroundColor = 0.157*[1 1 1];
                    obj.Parent.ForegroundColor = [1 1 1];
                end
                ax.Color = [0 0 0];
                grid(ax,'on');
                ax.GridColor = 0.68*[1 1 1];
                ax.GridAlpha = 0.4;
                axis(ax,'equal');
                ax.XColor = [1 1 1]*0.68;
                ax.YColor = [1 1 1]*0.68;
                ax.ZColor = [1 1 1]*0.68;
            else
                if isa(obj.Parent,'matlab.ui.container.Panel')
                    obj.Parent.BackgroundColor = [1 1 1];
                end
            end
            
            obj.Axes = ax;
            if obj.PanWithEgo || obj.PlotTruth || obj.PlotRoads
                obj.ScenarioPlotter = driving.scenario.Plot('EgoActor',egoVehicle,...
                    'Parent',ax,'Meshes','on',...
                    'ViewLocation',[obj.ViewLocation(1) obj.ViewLocation(2)],...
                    'ViewHeight',obj.ViewLocation(3),...
                    'ViewRoll',obj.ViewAngles(3),...
                    'ViewPitch',obj.ViewAngles(2),...
                    'ViewYaw',obj.ViewAngles(1));
                
                if obj.GroundTruthColorIndex > 0
                    for i = 2:numel(scenario.Actors)
                        scenario.Actors(i).PlotColor = obj.ColorOrder(obj.GroundTruthColorIndex,:);
                    end
                end
                
                l  = legend(ax);
                l.AutoUpdate = 'off';
                obj.ScenarioPlotter.plotActors(scenario.Actors)
                allPatches = findall(ax,'Type','Patch');
                allTags = arrayfun(@(x)x.Tag,allPatches,'UniformOutput',false);
                actorPatches = allPatches(startsWith(allTags,'ActorPatch'));
                
                
                if obj.PlotPointClouds
                    for i = 1:numel(actorPatches)
                        actorPatches(i).FaceAlpha = 0.3;
                        actorPatches(i).EdgeAlpha = 0.1;
                    end
                end
                
                if obj.PlotRoads
                    obj.ScenarioPlotter.plotRoads(scenario.RoadTiles, roadBoundaries(scenario),scenario.RoadCenters,scenario.ShowRoadBorders);
                    roadPatches = findall(ax,'Tag','RoadTilesPatch');
                    for i = 1:numel(roadPatches)
                        if obj.DarkMode
                            roadPatches(i).FaceColor = [0.1 0.1 0.1];
                            roadPatches(i).EdgeColor = [0 0 0];
                        end
                    end
                end
            end
            
            l.AutoUpdate = 'on';
            
            tp = theaterPlot('Parent',ax);
            bep = birdsEyePlot('Parent',ax);
            
            if obj.PlotTracks
                obj.TrackPlotter = trackPlotter(tp,'DisplayName','Tracks',...
                    'Marker','none',...
                    'MarkerFaceColor',obj.ColorOrder(obj.TrackColorIndex,:),...
                    'MarkerEdgeColor',obj.ColorOrder(obj.TrackColorIndex,:),...
                    'LabelOffset',[-3 0 2]);
                sampleTrk = struct('TrackID',nan,'State',nan(7,1),'StateCovariance',eye(7));
                [pos, vel, dim, orient, dims] = parseTracks(sampleTrk);
                obj.TrackPlotter.plotTrack(pos, vel, dim, orient, dims, {'nan'});
                h = findall(ax,'Tag','tpTrackExtent');
                h.LineWidth = 2;
                obj.TrackPlotterPatch = h;
            end
            
            if obj.PlotCoverage
                cvPlotters = cell(6,1);
                
                c = obj.ColorOrder;
                idx = [1 2 3 4 2 3];
                idx(:) = obj.CoverageColorIndex;
                cvColors = c(idx,:);
                
                for i = 1:6
                    if i == 1
                        displayName = 'Radar coverage';
                    else
                        displayName = '';
                    end
                    cvPlotters{i} = coverageAreaPlotter(bep,'FaceColor',cvColors(i,:),'EdgeColor',cvColors(i,:),....
                        'DisplayName',displayName,'FaceAlpha',0);
                end
                obj.CoveragePlotter = cvPlotters;
            end
            
            l.AutoUpdate = 'on';
            if obj.PlotPointClouds
                hold(ax,'on');
                obj.PointCloudPlotter = scatter3(ax,nan,nan,nan,'o','DisplayName','Point clouds',...
                    'SizeData',20,'MarkerFaceColor','flat');
            end
            
            l = legend(ax);
            l.TextColor = 1 - ax.Color;
            l.Location = 'best';
            colormap(ax,'parula');
            
            if ~obj.ShowLegend
                l.delete;
            end
        end
        
        function stepImpl(obj, scenario, egoVehicle, radars, ptCloud, tracker, tracks)
            if obj.PlotTruth
                obj.ScenarioPlotter.plotActors(scenario.Actors);
            elseif ~isempty(obj.ScenarioPlotter)
                obj.ScenarioPlotter.plotActors(driving.scenario.Actor.empty(0,1));
            end
            
            if obj.PlotCoverage
                updateCoveragePlotters(obj, radars, egoVehicle);
            end
            
            if obj.PlotTracks
                % Update track display
                pos = parseSensorModel(obj,radars{1},egoVehicle);
                if obj.ReferenceEgo
                    tracks = transformToEgo(tracks,egoVehicle);
                end
                [pos, vel, dim, orient, ids] = parseTracks(tracks,0);
                obj.TrackPlotter.plotTrack(pos,vel,dim,orient,ids);
            end
            
            if obj.PlotPointClouds
                % Plot point cloud. Assuming each ptCloud is reported in the
                % sensor's coordinate frame
                updatePointCloudPlotter(obj, radars(obj.SensorToPlot), egoVehicle, ptCloud(obj.SensorToPlot));
            end
        end
        
        function updatePointCloudPlotter(obj, radars, egoVehicle, ptClouds)
            xyz = zeros(3,0);
            for i = 1:numel(radars)
                thisPtCloud = ptClouds{i};
                [x, y, z] = sph2cart(deg2rad(thisPtCloud.Azimuth),deg2rad(thisPtCloud.Elevation),thisPtCloud.Range);
                loc = [x;y;z]';
                [pos, orient] = parseSensorModel(obj, radars{i}, egoVehicle);
                loc = rotatepoint(orient,loc)' + pos;
                xyz = [xyz loc];
            end
            cdata = interp1([0 4],[0 1],xyz(3,:),"linear",'extrap');
            cdata(cdata > 1) = 1;
            cdata(cdata < 0) = 0;
            cdata = [cdata 0 1];
            xyz = [xyz nan(3,2)];
            set(obj.PointCloudPlotter,'XData',xyz(1,:),'YData',xyz(2,:),'ZData',xyz(3,:),'CData',cdata);
        end
        
        function updateCoveragePlotters(obj, radars, egoVehicle)
            cvPlotters = obj.CoveragePlotter;
            for i = 1:numel(cvPlotters)
                [pos, orient, fov, maxRange] = parseSensorModel(obj, radars{i}, egoVehicle);
                ypr = eulerd(orient,'ZYX','frame');
                cvPlotters{i}.plotCoverageArea(pos(1:2)',5,ypr(1),diff(fov));
            end
        end
        
        function tf = isPublishing(~)
            tf = false;
            try
                s = numel(dbstack);
                tf = s > 6;
            catch
            end
            %  tf = true;
        end
        
        function [pos, orient, fov, maxRange] = parseSensorModel(obj, radar, ego)
            if ~obj.ReferenceEgo
                position = ego.Position;
                orientation = quaternion([ego.Yaw ego.Pitch ego.Roll],'eulerd','ZYX','frame');
                scene2plat = rotmat(orientation,'frame');
            else
                position = zeros(1,3);
                orientation = quaternion.ones(1);
                scene2plat = eye(3);
            end
            
            pos = position(:) + scene2plat'*radar.MountingLocation(:);
            
            plat2sens = quaternion(radar.MountingAngles,'eulerd','ZYX','frame');
            orient = plat2sens*orientation;
            
            fov = radar.FieldOfView(1)*[-1/2 1/2];
            
            maxRange = radar.RangeLimits(2);
        end
    end
end

function [pos, vel, dim, orient,ids] = parseTracks(tracks, h)
dim = struct('Length',1,'Width',1,'Height',1,'OriginOffset',[0 0 0]);
orient = quaternion.ones(numel(tracks),1);

if nargin == 1
    h = 0;
end

if isempty(tracks)
    pos = zeros(0,3);
    vel = zeros(0,3);
    dim = repmat(dim,0,1);
    orient = quaternion.empty(0,1);
    ids = string.empty(0,1);
    return;
end

state = horzcat(tracks.State);
pos = [state([1 3],:);zeros(1,numel(tracks))];
vel = [state([2 4],:);zeros(1,numel(tracks))];
pos = pos';
vel = vel';
for i = 1:numel(tracks)
    pos(i,3) = h;
    dim(i).Length = state(6,i);
    dim(i).Width = state(7,i);
    dim(i).Height = 0;
    dim(i).OriginOffset = [0 0 0];
    orient(i) = quaternion([state(5,i) 0 0],'eulerd','ZYX','frame');
end
ids = strcat("T",string([tracks.TrackID]));
end

function colorOrder = darkColorOrder
colorOrder = [1.0000    1.0000    0.0667
    0.0745    0.6235    1.0000
    1.0000    0.4118    0.1608
    0.3922    0.8314    0.0745
    0.7176    0.2745    1.0000
    0.0588    1.0000    1.0000
    1.0000    0.0745    0.6510];

colorOrder(8,:) = [1 1 1];
colorOrder(9,:) = [0 0 0];
colorOrder(10,:) = 0.7*[1 1 1];
end

function colorOrder = standardColorOrder
colorOrder = lines(7);
colorOrder(8,:) = [0 0 0];
colorOrder(9,:) = [1 1 1];
colorOrder(10,:) = 0.3*[1 1 1];
end


function tracksEgo = transformToEgo(tracks, ego)
tracksEgo = tracks;
for i = 1:numel(tracks)
    posGlobal = [tracks(i).State([1 3]);0];
%     velGlobal = [tracks(i).State([2 4]);0];
    yawGlobal = tracks(i).State(5);
    posEgo = ego.Position';
%     velEgo = ego.Velocity';
    yawEgo = ego.Yaw;
    q = quaternion([yawEgo 0 0],'eulerd','ZYX','frame');
    posWrtEgo = rotateframe(q,(posGlobal - posEgo)');
%     velWrtEgo = rotateframe(q,(velGlobal - velEgo)');
    yawWrtEgo = yawGlobal - yawEgo;
    tracksEgo(i).State([1 3]) = posWrtEgo(1:2);
    %    tracksEgo(i).State([2 4]) = velWrtEgo(1:2);
    tracksEgo(i).State(5) = yawWrtEgo;
end
end
