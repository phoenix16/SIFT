% Code reference: SIFT Matlab tutorial by Gonzalo Vaca-Castano

clear
clc
close all

source = imread('venice.jpg');
input = imread('to_be_matched.jpg');

% Initial parameter settings
num_octaves = 4;
num_scales = 5;
antialias_sigma = 0.5;
initSigma = sqrt(2);
contrast_threshold = 0.03;
r_curvature = 10;

% Convert images to grayscale if RGB
if (size(source,3))
   source = rgb2gray(source); 
end 
if (size(input,3))
   input = rgb2gray(input); 
end 

appended = append_images('LR', input,source);
% figure(1)
% imshow(appended);
clear appended


%%
tic
%=========================================================================%
%                    Scale Space and LoG approximation
%=========================================================================%

% Create a cell array to hold all the octaves and scales of the Scale Space
ScaleSpace = cell(num_octaves,num_scales);
DoG = cell(num_octaves,num_scales-1); % Scales 1, 2, 3, 4, 5 correspond to DoGs of scales 1, 2, 3, 4

% For octave 1, scale 1 image, source --> blur with sigma = 0.5 --> double size
source_antialiased = imfilter(source, fspecial('gaussian',[5 5], antialias_sigma));
ScaleSpace{1,1} =  imresize(source_antialiased, 2, 'bilinear');

for oct = 1:num_octaves
    sigma = initSigma;  % reset sigma for each octave    
    for sc = 1:num_scales-1
        sigma = sigma * 2^((sc-1)/2);
        % Apply blur to get next scale in same octave
        ScaleSpace{oct,sc+1} = imfilter(ScaleSpace{oct,sc}, fspecial('gaussian',[5 5], sigma));
        DoG{oct,sc} = ScaleSpace{oct,sc} - ScaleSpace{oct,sc+1};        
    end    
    
    % Create the next octave image if not reached the last octave
    % Next octave's first scale = prev octave's first scale downsized by half
    if (oct < num_octaves)
        ScaleSpace{oct+1,1} = imresize(ScaleSpace{oct,1}, 0.5, 'bilinear');   
    end    
end

%%
% Display the Scale Space images
app_SS = cell(num_octaves);
for oct = 1:num_octaves
    app_SS{oct} = append_images('LR',ScaleSpace{oct,1}, ScaleSpace{oct,2}, ScaleSpace{oct,3}, ScaleSpace{oct,4}, ScaleSpace{oct,5});
end
appended_SS = append_images('TB', app_SS{1}, app_SS{2}, app_SS{3}, app_SS{4});
% figure(2)
% imshow(appended_SS);


% Display the DoG images
app_DoGs = cell(num_octaves);
for oct = 1:num_octaves
    app_DoGs{oct} = append_images('LR',DoG{oct,1}, DoG{oct,2}, DoG{oct,3}, DoG{oct,4});
end
appended_DoGs = append_images('TB', app_DoGs{1}, app_DoGs{2}, app_DoGs{3}, app_DoGs{4});
% figure(3)
% imshow(appended_DoGs);
clear appended_DoGs appended_SS app_DoGs app_SS antialias_sigma initSigma source_antialiased sigma
toc
disp('    Created Scale Space');
   
%%
tic
%=========================================================================%
%                        Find Keypoints: DoG extrema
%=========================================================================%
%  n scales --> (n-1) DoGs --> (n-1)-2 = (n-3) DoGs with keypoints as 1st and last DoG scales in each octave do not have sufficient neighbors to be checked for extrema
% DoGs of Scales 2, 3, 4 correspond to DoG_Keypts of scales 1, 2
DoG_Keypts = cell(num_octaves,num_scales-3);

for oct = 1:num_octaves
    for sc = 2:num_scales-2
        DoG_Keypts{oct,sc-1} = DoG_extrema(DoG{oct,sc-1}, DoG{oct,sc}, DoG{oct,sc+1}); 
    end
end

toc
disp('    Found Keypoints = DoG extrema');


%%
tic
%=========================================================================%
%       Filter Keypoints: Remove low contrast and edge keypoints
%=========================================================================%
% DoGs of Scales 2, 3, 4 correspond to DoG_Keypts of scales 1, 2

for oct = 1:num_octaves
    for sc = 1:num_scales-3
        
        [x,y] = find(DoG_Keypts{oct,sc});  % indices of the Keypoints
        num_keypts = size(find(DoG_Keypts{oct,sc}));  % number of Keypoints
        level = DoG{oct,sc+1};
        
        for k = 1:num_keypts
            x1 = x(k);
            y1 = y(k);
            % Discard low contrast points
            if (abs(level(x1+1,y1+1)) < contrast_threshold)
               DoG_Keypts{oct,sc}(x1,y1) = 0;
            % Discard extrema on edges
            else
               rx = x1+1;
               ry = y1+1;
               % Get the elements of the 2x2 Hessian Matrix
               fxx = level(rx-1,ry) + level(rx+1,ry) - 2*level(rx,ry);   % double derivate in x direction
               fyy = level(rx,ry-1) + level(rx,ry+1) - 2*level(rx,ry);   % double derivate in y direction
               fxy = level(rx-1,ry-1) + level(rx+1,ry+1) - level(rx-1,ry+1) - level(rx+1,ry-1); % Partial derivate in x and y direction
               % Find Trace and Determinant of this Hessian
               trace = fxx + fyy;
               deter = fxx*fyy - fxy*fxy;
               curvature = trace*trace/deter;
               curv_threshold = ((r_curvature+1)^2)/r_curvature;
               if (deter < 0 || curvature > curv_threshold)   % Reject edge points if curvature condition is not satisfied
                   DoG_Keypts{oct,sc}(x1,y1 )= 0;
               end
            end
        end
    end
end

clear level trace deter x1 y1 rx ry fxx fxy fyy curv_threshold curvature contrast_threshold r_curvature
toc
disp('    Eliminated Keypoints with low contrast or on edges');

%%
tic
%=========================================================================%
%                   Assign Orientations to Keypoints
%=========================================================================%
% Allocate memory for the gradient magnitude and orientations
grad_mag = cell(size(ScaleSpace));
grad_angle = cell(size(ScaleSpace));

% Assign orientations & magnitude to the Scale Space images (all points, not just keypoints)
for oct = 1:num_octaves
    for sc = 1:num_scales
        level = ScaleSpace{oct, sc};
        
        A = level(1:end-2,2:end-1);
        B = level(3:end, 2:end-1);
        C = level(2:end-1,1:end-2);
        D = level(3:end, 3:end);
        grad_mag{oct,sc} = sqrt(double(((A-B).^2) + ((C-D).^2)));
        grad_angle{oct,sc} = atan(double((A-B)./(C-D))); 
    end
end

clear A B C D
toc
disp('    Assigned Orientations');





