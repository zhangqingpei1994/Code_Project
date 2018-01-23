       function [ TBase, TEnd, cameraParams, TBaseStd, TEndStd, pixelErr ] = CalCamArm( imageFolder, armMat, squareSize, varargin )
%CALCAMARM Calibrates a camera to work with a robotic arm by finding the
%camera intrinsics and the camera to arm base transformation matrix using a
%series of arm poses and corresponding images of the arm holding a
%checkerboard. Assumes that the camera is rigidly mounted with respect to
%the arms base.
%
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   imageFolder- path to a folder containing n images of the robotic arm
%       holding a checkerboard taken by the camera you wish to calibrate. 
%       The entire board must be visible and rigidly attached to the
%       arm. The file names do not matter as long as when they are arranged
%       in alphabetical order they correspond to the order of the
%       transformations in armMat.
%
%   armMat- 4x4xn set of arm transformation matrices that give the
%       transformation from the arm's base to the end effector. (The end
%       effector to checkerboard Tform is automatically found)
%
%   squareSize- The width of one of the checkerboard's squares in mm.
%
%--------------------------------------------------------------------------
%   Optional Name-Value Pair Input Arguments:
%--------------------------------------------------------------------------
%   Verbose- true (default) | logical scalar
%       if true prints current stage of calibration
%   
%   outputTmat- true (default) | logical scalar
%       if true gives results as transformation matrices.
%       if false gives results as 1x6 vector with rotation in angle axis
%       format [x,y,z,rx,ry,rz]
%   
%   maxBaseOffset- 1 (default) | positive double
%       the maximum possible translational offset between the camera and
%       arm base in metres, if in doubt over-estimate.
%
%   maxEndOffset- 1 (default) | positive double
%       the maximum possible translational offset between the arms end
%       effector and the chessboard in metres, if in doubt over-estimate.
%
%   inliers- 80 (default) | double in range 0 to 100
%       percent of data to take as inliers, helps protect against a
%       misaligned board slipping through and messing the results up.
%
%   errEst- true (default) | logical scalar
%       if true bootstraps the data to give estimate of calibration
%       accuracy. If false output Std will be zero
%
%   numBoot- 100 (default) | positive integer
%       number of times to bootstrap data, only used if errEst is true
%
%   cameraParams- cameraParameters object
%       if given the calibration will use these camera parameters. If not
%       given the camera parameters are generated from the data.
%
%   baseEst- 4x4 identity matrix (default) | 4x4 double matrix
%       4x4 matrix giving the initial estimate for the camera to base tform
%
%   endEst- 4x4 identity matrix (default) | 4x4 double matrix
%       4x4 matrix giving the initial estimate for the end effector to 
%       checekerboard tform
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   TBase- The camera to arm base transformation
%
%   TEnd- The end effector to checkerboard transformation
%
%   cameraParams- The intrinsic parameters of the camera used
%
%   TBaseStd- An estimation of the standard deviation of the error in the
%       values in TBase
%
%   TBaseStd- An estimation of the standard deviation of the error in the
%       values in TEnd
%
%   pixelErr- Mean error of projected inliers in pixels
%
%--------------------------------------------------------------------------
%   References:
%--------------------------------------------------------------------------
%   The implementation differs from but was inspired by the method
%   presented at http://robolabwiki.sdu.dk/mediawiki/index.php/Calibration_of_a_robotic_arm
%
%   This code was written by Zachary Taylor
%   z.taylor@acfr.usyd.edu.au
%   http://www.zjtaylor.com

%% check inputs

%get values for required inputs
validateattributes(imageFolder, {'char'},{});
if(exist(imageFolder,'dir') ~= 7)
    error('imageFolder must be a directory');
end

numImages = dir(imageFolder);
numImages = length({numImages(~[numImages.isdir]).name});  
validateattributes(armMat, {'numeric'},{'size',[4,4,numImages]});
armMat = double(armMat);

validateattributes(squareSize, {'numeric'},{'scalar'});
squareSize = double(squareSize);

%set optional inputs in default values
verbose = true;
outputTmat = true;
maxBaseOffset = 0.5;
maxEndOffset = 0.8;
inliers = 80;
errEst = false;
numBoot = 100;
%534.569,327.8929,252.7114;0,533.7478,0;0,0,1
cameraParams = [];
baseEst = eye(4);
 baseEst(3,4)=1.2;
 baseEst(2,4)=0.0;
 baseEst(1,4)=0.0;


endEst = eye(4);



%% Extract chessboards
if(verbose)
    fprintf('Starting Arm Calibration\n'); 
    tic;
end

%convert squareEst to metres
squareSize = squareSize / 1000;

%get images
imageFiles = dir(imageFolder);
imageFiles = {imageFiles(~[imageFiles.isdir]).name};
for i = 1:length(imageFiles)
    imageFiles{i} = [imageFolder filesep imageFiles{i}];
end

if(verbose)
    fprintf('Extracting Chessboards\n');
end

%find checkerboards
[points, boardSize, imagesUsed] = detectCheckerboardPoints(imageFiles);  %  points:88*2*26


    
if(sum(imagesUsed) == 0)
    error('No checkerboards were found in the images');
elseif(sum(imagesUsed) < 10)
    warning(['only %i checkerboards found.\n',...
        '10 images is the minimum number of boards recommended for an accurate calibration (though more is better, orignal calibration used 50+)\n',...
        'Check your images and keep in mind, lighting, occlusions and backgrounds with roughly checkered patterns'], tValid);
end
display(sum(imagesUsed));

%% Process arm poses

%remove unused poses
armPose = armMat(:,:,imagesUsed);

%% Find camera parameters
if(verbose)
    fprintf('Finding Camera Parameters\n');
end

%generate an ideal chessboard to compare points to
worldPoints = generateCheckerboardPoints(boardSize, 1);

%estimate camera parameters
if(isempty(cameraParams))
    cameraParams = estimateCameraParameters(points,squareSize*worldPoints,'WorldUnits','m','NumRadialDistortionCoefficients',3,'EstimateTangentialDistortion',true);
end
    
%% Optimize
if(verbose)
    fprintf('Running Optimization\n');
end

%estimate for camera to arm base transform
baseEst = T2V(baseEst);
%size of range to search around above estimate for the true value
baseRange = [maxBaseOffset,maxBaseOffset,maxBaseOffset,pi,pi,pi];

%estimate for gripper to chessboard transofrm
endEst = T2V(endEst);
%size of range to search around above estimate for the true value
gripRange = [maxEndOffset,maxEndOffset,maxEndOffset,pi,pi,pi];

%size of range to search around above estimate for the true value
squareRange = 0.001;

%set up search range
inital = [baseEst,endEst,squareSize];

ub = [baseEst,endEst,squareSize] + [baseRange,gripRange,squareRange];
lb = [baseEst,endEst,squareSize] - [baseRange,gripRange,squareRange];

%set to intrior-point to allow for gradient free optimization
options = optimoptions(@fmincon,'Algorithm','interior-point','Display','off');

%function to optimize
optFunc = @(est) ProjectError(points, cameraParams, worldPoints, armPose, inliers, est);

%optimize
[solution,pixelErr] = fmincon(optFunc,inital,[],[],[],[],lb,ub,[],options);

if(pixelErr > 10)
    warning(['Average projection error found to be %3.1f pixels.\n',...
        'This large error is a strong indicator that something has gone wrong.\n',...
        'Check that the input paramters are correct and sufficient checkerboard images were correctly processed.\n',...
        'If the problem persists, try manually tuning the baseEst and gripEst parameters.\n',...
        'If it still will not work, email me a z.taylor@acfr.usyd.edu.au\n'], pixelErr);
end

%% Bootstrap
if(errEst)
    bootSol = zeros(numBoot,length(inital));
    if(verbose)
        fprintf('Running Bootstrap Optimization       ');
    end
    for idx = 1:numBoot
        %sample points
        sample = datasample(1:size(armPose,3),size(armPose,3));
        bootArmPose = armPose(:,:,sample);
        bootPoints = points(:,:,sample,:);

        %function to optimize
        optFunc = @(est) ProjectError(bootPoints, cameraParams, worldPoints, bootArmPose, inliers, est);

        %optimize
        bootSol(idx,:) = fmincon(optFunc,solution,[],[],[],[],lb,ub,[],options);
        bootSol = std(bootSol);
                    
        if(verbose)
            fprintf('\b\b\b\b\b\b\b %5.1f%%',100*idx/numBoot);
        end
            
    end
    
    if(verbose)
        fprintf('\b\b\b\b\b\b\b \n');
    end
else
    bootSol = zeros(1,13);
end

%% Convert format
if(verbose)
    fprintf('Converting to Transformation matricies\n');
end

%angle axis form
TBase = solution(1,1:6);
TBaseStd = bootSol(:,1:6);
TEnd = solution(1,7:12);
TEndStd = bootSol(:,7:12);

%convert to matrix
if(outputTmat)
    [ TBase, TBaseStd ] = ConvertTformSystem(TBase, TBaseStd);
    [ TEnd, TEndStd ] = ConvertTformSystem(TEnd, TEndStd);
end

if(verbose)
    fprintf('Calibration completed in %3.1f seconds with a mean error of %1.3f pixels\n',toc,pixelErr);
end
