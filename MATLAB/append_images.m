function appended = append_images(mode, varargin)
% Function to append images Left-Right or Top-Bottom
%       mode = 'LR' or 'TB' for left-right or top-bottom concatenation
% Rows (or cols) at the bottom (or right) of the smaller images are zero padded

% Create a copy of input images before messing with them 
% (else would be pass by reference)
images = cell(1,nargin); images = varargin; 

% dim_array corresponds to rows if 'LR' and cols if 'TB'
dim_array = zeros(1,nargin-1);
for n = 1:(nargin-1)
    if strcmp(mode,'LR')
        dim_array(n) = size(images{n},1);
    else
        dim_array(n) = size(images{n},2);
    end   
end
max_dim = max(dim_array);
appended = [];

for n = 1:nargin-1
    switch mode 
        case 'LR'  % Pad zeros at the bottom of smaller images
            if size(images{n},1) < max_dim
                images{n}(max_dim,1) = 0;
            end
            appended = [appended   images{n}];
        case 'TB'  % Pad zeros at the right of smaller images
            if size(images{n},2) < max_dim
                images{n}(1,max_dim) = 0;
            end   
            appended = [appended ; images{n}];   
    end
end
end