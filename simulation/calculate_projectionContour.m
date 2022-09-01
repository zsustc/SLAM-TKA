function [contour, contour_index] = ...
    calculate_projectionContour(vertices,holeThreshold,edge_ratio_top,edge_ratio_bottom)
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
%calculate projection contour using projection vertices
% contour: u, v-->v,u

shp = alphaShape(vertices(:,1),vertices(:,2), 'HoleThreshold',holeThreshold); % the value of obj.alphaShape 
[~,P] = boundaryFacets(shp);
boundary_index = zeros(size(P,1),1);
for ll=1:size(P,1)
    id_tem = find(vertices(:,1)==P(ll,1) & vertices(:,2)==P(ll,2));
    
    if (size(id_tem,1)==1)
        boundary_index(ll) = id_tem;   % only select the points that is not overlap
    else
        boundary_index(ll) = NaN;      % use a simple way to remove the redundant boundary
    end
end
boundary_index(isnan(boundary_index)) = [];  % remove nan

contour_index = boundary_index;
contour = vertices(contour_index,1:2);        % contour of model


[~, sort_index]= sort(contour(:,2),1); % sort the rows of the contour
contour = contour(sort_index,:); % sort the contour by index
contour_index = contour_index(sort_index,:);

% remove the top and bottom of contour
num = size(contour,1);
if num == 0
    contour = zeros(0,2);
    fprintf('contour is invalid');
else
    contour = contour(round(1+edge_ratio_top*num):round(edge_ratio_bottom*num), :);
    contour_index = contour_index(round(1+edge_ratio_top*num):round((1-edge_ratio_top)*num), :);
end

% sawp u and v
u = contour(:,1);
v = contour(:,2);
contour = [v,u];

contour_without_boders = zeros(0,2);
%% filter the border edge points
num_contour = size(contour,1);
rows = 1024;
cols = 1024;
if num_contour == 0
    contour_without_boders = zeros(0,2);
else
    for i = 1: num_contour
        vv = contour(i,1);
        uu = contour(i,2);
        if  vv > 4 && vv < rows-4 && uu > 4 && uu < cols -4
            contour_without_boders = [contour_without_boders;vv,uu];
        end
    end
end


end