function keypt_locs = DoG_extrema_Slow(top_scale, current_scale, bottom_scale)
% Function to find the keypoints and their locations given 3 DoG scales
% A pixel in current scale = Keypoint if it is the extremum when compared
% to its 8 neighbors in current scale and 9 neighbors each in top and
% bottom scales

d_curr = [ 1 0; -1 0; 1 1; 0 1; -1 1; 1 -1; 0 -1; -1 -1]; % indices of 8 neighbors
d_adj = [ 0 0; 1 0; -1 0; 1 1; 0 1; -1 1; 1 -1; 0 -1; -1 -1]; % indices of 8 neighbors + self
num_rows = size(current_scale,1);
num_cols = size(current_scale,2);

% Initialize array of Keypoint locations
keypt_locs = zeros(num_rows*num_cols,2); % conservative estimate the less than quarter of the pixels in the scale are key points
p = 1;

for row = 2:num_rows-1
    for col = 2:num_cols-1
        loc = [row col];       
        curr_ns = d_curr + repmat(loc, [8 1]);  % curr_ns = row & col indices of 8 neighbors in current scale
        adj_ns  = d_adj + repmat(loc, [9 1]); % adj_ns  = row & col indices of 9 neighbors in adjacent scale
        
        % Store the 8 neighbors around it and 9 neighbors in the scale above and below it
        neighbors_8  = diag(current_scale(curr_ns(:,1),curr_ns(:,2)));
        neighbors_t9 = diag(top_scale(adj_ns(:,1),adj_ns(:,2)));
        neighbors_b9 = diag(bottom_scale(adj_ns(:,1),adj_ns(:,2)));

        % If the current pixel is an extremum compared to its 8 neighbors, save its location
        if (current_scale(row,col) < min(neighbors_8)) || (current_scale(row,col) > max(neighbors_8))
            keypt_locs(p,:) = [row col];  p = p+1;
        elseif (current_scale(row,col) < min(neighbors_t9)) || (current_scale(row,col) > max(neighbors_t9))
            keypt_locs(p,:) = [row col];  p = p+1;
        elseif (current_scale(row,col) < min(neighbors_b9)) || (current_scale(row,col) > max(neighbors_b9))
            keypt_locs(p,:) = [row col];  p = p+1;
        end       

        if (p == num_rows*num_cols/2)
            disp('Keypoints container overflow!!!');
        end
    end
end

% keypts = diag(current_scale(keypt_locs(:,1),keypt_locs(:,2)));

end