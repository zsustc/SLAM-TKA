
function [projected_image, projected_points_index] = ...
    pointCloud_perspective_projection(points_camera_space,K,rows,cols)
% ===============================================================================================
% SLAM-TKA: Real-time Intra-operative Measurement of Tibial Resection Plane in Conventional Total Knee Arthroplasty 
% Version: 1.0
% ===============================================================================================
% 
% Copyright (C) 2013 Shuai Zhang, Liang Zhao, Shoudong Huang
% University of Technology, Sydney, Australia
% 
% Authors:  Shuai Zhang -- Shuai.Zhang@student.uts.edu.au
%           Liang Zhao         -- Liang.Zhao-1@uts.edu.au 
%           Shoudong Huang     -- Shoudong.Huang@uts.edu.au
%           
% 
%           Robotics Institute
%           Faculty of Engineering and Information Technology
%           University of Technology, Sydney
%           NSW 2022, Australia
% 
% 
% Please contact Shuai Zhang {Shuai.Zhang@student.uts.edu.au} if you have any questions/comments about the code.
% 
% points_camera_space is 3-by-n points array in the camera space
% K is the camera intrinsic parameters
% rows is the width of the projected image
% cols is the height of the projected image
% projected_image is the projected pixel image with the intensity 1 for
% each projected pixel
% depth_image is the matrix storing point depth

projected_image = zeros(rows, cols);% define an image plane with size [rows, cols]
projected_points_index = zeros(rows, cols);
points_camera_space = points_camera_space';

for i = 1:size(points_camera_space,1)
    p_c = points_camera_space(i,:);
    p = K * p_c';

    depth_value = p(3);
    if (depth_value < 0)
        continue;
    end
    u = p(1)/p(3); % normalize
    v = p(2)/p(3); % image plane is behind the object?
    u = round(u);
    v = round(v);
    if u >= 1 && u <= cols && v >= 1 && v <= rows
        projected_image(v,u) = 1;
        projected_points_index(v,u) = i;
    end
end

end