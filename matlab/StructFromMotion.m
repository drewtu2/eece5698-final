%% Load Images into memory...
% Use |imageDatastore| to get a list of all image file names in a
% directory.
calibDir = '../';
imageDir = '../../eece5698/lab4/latino_center/';
imageDir = '../room/';
imds = imageDatastore(imageDir);
scale = [4 4 1];
I = readimage(imds, 1);
inputSize = size(I)./scale
imds.ReadFcn = @(buildingScene)imresize(imread(buildingScene), inputSize(1:2));
numPoints = 1000;
tiles = [1 8];

% Display the images we're meshing...
% figure
% montage(imds.Files, 'Size', [4, 5]);

% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
    I = readimage(imds, i);
    images{i} = rgb2gray(I);
end

% Create an empty viewSet object to manage the data associated with each
% view.
vSet = viewSet;

load(fullfile(calibDir, 'cameraParams.mat'));

%% Calculate first set of features

% Undistort the first image.
I = undistortImage(images{1}, cameraParams);

% Detect features. Increasing 'NumOctaves' helps detect large-scale
% features in high-resolution images. Use an ROI to eliminate spurious
% features around the edges of the image.
border = 50;
roi = [border, border, size(I, 2)- 2*border, size(I, 1)- 2*border];

%prevPoints   = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);
[yp, xp, ~] = harris(I, numPoints, 'tile', tiles);
prevPoints = cornerPoints([xp yp]);

% Extract features. Using 'Upright' features improves matching, as long as
% the camera motion involves little or no in-plane rotation.
[prevFeatures, prevPoints]= extractFeatures(I, prevPoints);

% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', ...
    eye(3, 'like', prevPoints.Location), 'Location', ...
    zeros(1, 3, 'like', prevPoints.Location));

for i = 2:numel(images)
    % Undistort the current image.
    %I = undistortImage(images{i}, cameraParams);
    I = images{i};
    
    % Detect, extract and match features.
    % currPoints   = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);
        
    % Initialize features for I(1)
    %points = detectSURFFeatures(grayImage);
    [yp, xp, ~] = harris(I, numPoints, 'tile', tiles);
    currPoints = cornerPoints([xp yp]);

    [currFeatures, currPoints] = extractFeatures(I, currPoints);
    
%     indexPairs = matchFeatures(prevFeatures, currFeatures, ...
%         'MaxRatio', .9, 'Unique',  true);
    indexPairs = matchFeatures(prevFeatures, currFeatures);


    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
    
    figure;
    showMatchedFeatures(images{i-1}, images{i}, matchedPoints1, matchedPoints2);
    title('Tracked Features');
                                    

    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    
    [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(matchedPoints1, matchedPoints2, cameraParams);

    % Add the current view to the view set.
    vSet = addView(vSet, i, 'Points', currPoints);

    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, 'Matches', indexPairs(inlierIdx,:));

    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1);
    prevOrientation = prevPose.Orientation{1};
    prevLocation    = prevPose.Location{1};

    % Compute the current camera pose in the global coordinate system
    % relative to the first view.
    orientation = relativeOrient * prevOrientation;
    location    = prevLocation + relativeLoc * prevOrientation;
    vSet = updateView(vSet, i, 'Orientation', orientation, ...
        'Location', location);

    % Find point tracks across all views.
    tracks = findTracks(vSet);

    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);

    % Refine the 3-D world points and camera poses.
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
        'PointsUndistorted', true);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints   = currPoints;
end

%% Display camera poses.
camPoses = poses(vSet);
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on

% Exclude noisy 3-D points.
goodIdx = (reprojectionErrors < 5);
xyzPoints = xyzPoints(goodIdx, :);

% Display the 3-D points.
pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

title('Refined Camera Poses');

% Read and undistort the first image
I = undistortImage(images{1}, cameraParams);

% Detect corners in the first image.
prevPoints = detectMinEigenFeatures(I, 'MinQuality', 0.001);

% Create the point tracker object to track the points across views.
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 6);

% Initialize the point tracker.
prevPoints = prevPoints.Location;
initialize(tracker, prevPoints, I);

% Store the dense points in the view set.
vSet = updateConnection(vSet, 1, 2, 'Matches', zeros(0, 2));
vSet = updateView(vSet, 1, 'Points', prevPoints);

% Track the points across all views.
for i = 2:numel(images)
    % Read and undistort the current image.
    I = undistortImage(images{i}, cameraParams);

    % Track the points.
    [currPoints, validIdx] = step(tracker, I);

    % Clear the old matches between the points.
    if i < numel(images)
        vSet = updateConnection(vSet, i, i+1, 'Matches', zeros(0, 2));
    end
    vSet = updateView(vSet, i, 'Points', currPoints);

    % Store the point matches in the view set.
    matches = repmat((1:size(prevPoints, 1))', [1, 2]);
    matches = matches(validIdx, :);
    vSet = updateConnection(vSet, i-1, i, 'Matches', matches);
end

% Find point tracks across all views.
tracks = findTracks(vSet);

% Find point tracks across all views.
camPoses = poses(vSet);

% Triangulate initial locations for the 3-D world points.
xyzPoints = triangulateMultiview(tracks, camPoses,...
    cameraParams);

% Refine the 3-D world points and camera poses.
[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(...
    xyzPoints, tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
    'PointsUndistorted', true);

% Display the refined camera poses.
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on

% Exclude noisy 3-D world points.
goodIdx = (reprojectionErrors < 5);

% Display the dense 3-D world points.
pcshow(xyzPoints(goodIdx, :), 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

title('Dense Reconstruction');