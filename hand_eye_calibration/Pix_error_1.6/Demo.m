

load TCP_position.txt;
         
armMat=rand(4,4,15);
 
 for i=1:15
   armMat(1:3,1:3,i)=Trans_vect_to_matrix(TCP_position(i*2-1,:));
   armMat(1:3,4,i)=TCP_position(i*2,:);
   armMat(4,:,i)=0;
   armMat(4,4,i)=1;
 end
 
save 'Example Data/armMat.mat' armMat;


imageFolder = './Example Data/Images/';
%loading arm transformations
load('./Example Data/armMat.mat');
%checkerboard square widths in mm
squareSize = 15;

%run calibration
[TBase, TEnd, cameraParams, TBaseStd, TEndStd, pixelErr] = CalCamArm(imageFolder, armMat, squareSize,'maxBaseOffset',0.5);

%print results
fprintf('\nFinal camera to arm base transform is\n')
disp(TBase);

fprintf('Final end effector to checkerboard transform is\n')
disp(TEnd);

fprintf('Final camera matrix is\n')
disp(cameraParams.IntrinsicMatrix');

fprintf('Final camera radial distortion parameters are\n')
disp(cameraParams.RadialDistortion);

% fprintf('Final camera tangential distortion parameters are\n')
disp(cameraParams.TangentialDistortion);