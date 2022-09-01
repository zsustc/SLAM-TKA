function [Error,Sum_Error,Jacobian] = ...
    FuncDiffJacobianMultiViews(K,rows,cols,TibiaGrid,pinGrid,X,CovMatrixInv,...
    uv_edge_tibia_first_view_gt,uv_edge_tibia_second_view_gt,uv_edge_left_pin_first_view_gt,...
    uv_edge_left_pin_second_view_gt,uv_edge_right_pin_first_view_gt,uv_edge_right_pin_second_view_gt,...
    BplanKneeEdgeGrid,BplanPinEdgeGrid)

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
roi_bone_nearCamera = TibiaGrid.roi_bone_nearCamera.Location;
TibiaDTXgrid = TibiaGrid.TibiaDTXgrid;
TibiaDTYgrid = TibiaGrid.TibiaDTYgrid;
TibiaDTZgrid = TibiaGrid.TibiaDTZgrid;
TibiaGDTXXgrid = TibiaGrid.TibiaGDTXXgrid;
TibiaGDTXYgrid = TibiaGrid.TibiaGDTXYgrid;
TibiaGDTXZgrid = TibiaGrid.TibiaGDTXZgrid;
TibiaGDTYXgrid = TibiaGrid.TibiaGDTYXgrid;
TibiaGDTYYgrid = TibiaGrid.TibiaGDTYYgrid;
TibiaGDTYZgrid = TibiaGrid.TibiaGDTYZgrid;
TibiaGDTZXgrid = TibiaGrid.TibiaGDTZXgrid;
TibiaGDTZYgrid = TibiaGrid.TibiaGDTZYgrid;
TibiaGDTZZgrid = TibiaGrid.TibiaGDTZZgrid;

nail_points = pinGrid.nail_points;
PinDTXgrid = pinGrid.PinDTXgrid;
PinDTYgrid = pinGrid.PinDTYgrid;
PinDTZgrid = pinGrid.PinDTZgrid;
PinGDTXXgrid = pinGrid.PinGDTXXgrid;
PinGDTXYgrid = pinGrid.PinGDTXYgrid;
PinGDTXZgrid = pinGrid.PinGDTXZgrid;
PinGDTYXgrid = pinGrid.PinGDTYXgrid;
PinGDTYYgrid = pinGrid.PinGDTYYgrid;
PinGDTYZgrid = pinGrid.PinGDTYZgrid;
PinGDTZXgrid = pinGrid.PinGDTZXgrid;
PinGDTZYgrid = pinGrid.PinGDTZYgrid;
PinGDTZZgrid = pinGrid.PinGDTZZgrid;

KneeEdgeDTgrid = BplanKneeEdgeGrid.KneeEdgeDTgrid;
KneeEdgeGXgrid = BplanKneeEdgeGrid.KneeEdgeGXgrid;
KneeEdgeGYgrid = BplanKneeEdgeGrid.KneeEdgeGYgrid;
%KneeEdgeIDgrid = BplanKneeEdgeGrid.KneeEdgeIDgrid;
%KneeEdgeNV_theta = BplanKneeEdgeGrid.KneeEdgeNV_theta;

PinEdgeDTgrid = BplanPinEdgeGrid.PinEdgeDTgrid;
PinEdgeGXgrid = BplanPinEdgeGrid.PinEdgeGXgrid;
PinEdgeGYgrid = BplanPinEdgeGrid.PinEdgeGYgrid;
%PinEdgeIDgrid = BplanPinEdgeGrid.PinEdgeIDgrid;
%PinEdgeNV_theta = BplanPinEdgeGrid.PinEdgeNV_theta;

num_tibia_edge_first_view = size(uv_edge_tibia_first_view_gt,2);
num_tibia_edge_second_view = size(uv_edge_tibia_second_view_gt,2);

num_edge_left_pin_first_view = size(uv_edge_left_pin_first_view_gt,2);
num_edge_left_pin_second_view = size(uv_edge_left_pin_second_view_gt,2);

num_edge_right_pin_first_view = size(uv_edge_right_pin_first_view_gt,2);
num_edge_right_pin_second_view = size(uv_edge_right_pin_second_view_gt,2);

%% project the knee model and pin model using estimated pose
Alpha_first_view = X(1);
Beta_first_view = X(2);
Gamma_first_view = X(3);

R_cw_first_view = RMatrixYPR22(Alpha_first_view,Beta_first_view,Gamma_first_view); % convert to rotation as order of Alpha, Beta and Gamma
t_cw_first_view = X(4:6,1);

ptKneeModelfirstView = R_cw_first_view*roi_bone_nearCamera' + t_cw_first_view;
[knee_projected_image_first_view, projected_knee_points_index_first_view] = ...
    pointCloud_perspective_projection(ptKneeModelfirstView,K,rows,cols);

% knee edge extraction from model projection under first view
[ver_row, ver_col] = find(knee_projected_image_first_view == 1);
vertices = [ver_col, ver_row];
holeThreshold = 250;
edge_ratio_top = 0/10;
edge_ratio_bottom = 10/10;
[knee_edge_index_first_view, ~] = calculate_projectionContour(vertices,holeThreshold,edge_ratio_top,edge_ratio_bottom);

kneeModelObs2D_first_view = zeros(0,2);
kneeModelObs3D_first_view = zeros(0,3);
kneeModelObs3DCam_first_view = zeros(0,3);

for i = 1:size(knee_edge_index_first_view,1)
    v_row = knee_edge_index_first_view(i,1);
    u_col = knee_edge_index_first_view(i,2);
    index_tmpt = projected_knee_points_index_first_view(v_row, u_col);
    if index_tmpt > 0
        kneeModelObs2D_first_view = [kneeModelObs2D_first_view;v_row,u_col];
        kneeModelObs3D_first_view = [kneeModelObs3D_first_view;roi_bone_nearCamera(index_tmpt,:)];
        kneeModelObs3DCam_first_view = [kneeModelObs3DCam_first_view;ptKneeModelfirstView(:,index_tmpt)'];
    end
end

% edge extration from knee model projection under second view frame
index_X = 6+3*num_tibia_edge_first_view;

Alpha_second_view = X(index_X+1,1);
Beta_second_view = X(index_X+2,1);
Gamma_second_view = X(index_X+3,1); 
R_cw_second_view = RMatrixYPR22(Alpha_second_view,Beta_second_view,Gamma_second_view); % convert to rotation as order of Alpha, Beta and Gamma
t_cw_second_view = X(index_X+4:index_X+6,1);

ptKneeModelsecondView = R_cw_second_view*roi_bone_nearCamera' + t_cw_second_view;
[knee_projected_image_second_view, projected_knee_points_index_second_view] = ...
    pointCloud_perspective_projection(ptKneeModelsecondView,K,rows,cols);

[ver_row, ver_col] = find(knee_projected_image_second_view == 1);
vertices = [ver_col, ver_row];
holeThreshold = 250;
edge_ratio_top = 0/10;
edge_ratio_bottom = 10/10;
[knee_edge_index_second_view, ~] = calculate_projectionContour(vertices,holeThreshold,edge_ratio_top,edge_ratio_bottom);

kneeModelObs2D_second_view = zeros(0,2);
kneeModelObs3D_second_view = zeros(0,3);
kneeModelObs3DCam_second_view = zeros(0,3);

for i = 1:size(knee_edge_index_second_view,1)
    v_row = knee_edge_index_second_view(i,1);
    u_col = knee_edge_index_second_view(i,2);
    index_tmpt = projected_knee_points_index_second_view(v_row, u_col);
    if index_tmpt > 0
        kneeModelObs2D_second_view = [kneeModelObs2D_second_view;v_row,u_col];
        kneeModelObs3D_second_view = [kneeModelObs3D_second_view;roi_bone_nearCamera(index_tmpt,:)];
        kneeModelObs3DCam_second_view = [kneeModelObs3DCam_second_view;ptKneeModelsecondView(:,index_tmpt)'];
    end
end

% edge extration from left pin model projection under first view frame
index_X = 6+3*num_tibia_edge_first_view+6+3*num_tibia_edge_second_view;

Alpha_left_pin = X(index_X+1,1);
Beta_left_pin = X(index_X+2,1);
Gamma_left_pin = X(index_X+3,1); 
R_wn_left_pin = RMatrixYPR22(Alpha_left_pin,Beta_left_pin,Gamma_left_pin); % convert to rotation as order of Alpha, Beta and Gamma
t_wn_left_pin = X(index_X+4:index_X+6,1);

ptLeftPinModelCamerafirstView = ...
    R_cw_first_view*( R_wn_left_pin*nail_points' + t_wn_left_pin) + t_cw_first_view;

[left_pin_projected_image_first_view, projected_left_pin_points_index_first_view] = ...
    pointCloud_perspective_projection(ptLeftPinModelCamerafirstView,K,rows,cols);

[ver_row, ver_col] = find(left_pin_projected_image_first_view == 1);
vertices = [ver_col, ver_row];
holeThreshold = 50;
edge_ratio_top = 0/10;
edge_ratio_bottom = 10/10;
[left_pin_edge_index_first_view, ~] = calculate_projectionContour(vertices,holeThreshold,edge_ratio_top,edge_ratio_bottom);

leftPinModelObs2D_first_view = zeros(0,2);
leftPinModelObs3D_first_view = zeros(0,3);
leftPinModelObs3DCam_first_view = zeros(0,3);

for i = 1:size(left_pin_edge_index_first_view,1)
    v_row = left_pin_edge_index_first_view(i,1);
    u_col = left_pin_edge_index_first_view(i,2);
    index_tmpt = projected_left_pin_points_index_first_view(v_row, u_col);
    if index_tmpt > 0
        leftPinModelObs2D_first_view = [leftPinModelObs2D_first_view;v_row,u_col];
        leftPinModelObs3D_first_view = [leftPinModelObs3D_first_view;nail_points(index_tmpt,:)];
        leftPinModelObs3DCam_first_view = ...
            [leftPinModelObs3DCam_first_view;ptLeftPinModelCamerafirstView(:,index_tmpt)'];
    end
end

% edge extration from left pin model projection under second view frame
ptLeftPinModelCamerasecondView = ...
    R_cw_second_view*( R_wn_left_pin*nail_points' + t_wn_left_pin) + t_cw_second_view;
[left_pin_projected_image_second_view, projected_left_pin_points_index_second_view] = ...
    pointCloud_perspective_projection(ptLeftPinModelCamerasecondView,K,rows,cols);

[ver_row, ver_col] = find(left_pin_projected_image_second_view == 1);
vertices = [ver_col, ver_row];
holeThreshold = 50;
edge_ratio_top = 0/10;
edge_ratio_bottom = 10/10;
[left_pin_edge_index_second_view, ~] = calculate_projectionContour(vertices,holeThreshold,edge_ratio_top,edge_ratio_bottom);

leftPinModelObs2D_second_view = zeros(0,2);
leftPinModelObs3D_second_view = zeros(0,3);
leftPinModelObs3DCam_second_view = zeros(0,3);

for i = 1:size(left_pin_edge_index_second_view,1)
    v_row = left_pin_edge_index_second_view(i,1);
    u_col = left_pin_edge_index_second_view(i,2);
    index_tmpt = projected_left_pin_points_index_second_view(v_row, u_col);
    if index_tmpt > 0
        leftPinModelObs2D_second_view = [leftPinModelObs2D_second_view;v_row,u_col];
        leftPinModelObs3D_second_view = [leftPinModelObs3D_second_view;nail_points(index_tmpt,:)];
        leftPinModelObs3DCam_second_view = ...
            [leftPinModelObs3DCam_second_view;ptLeftPinModelCamerasecondView(:,index_tmpt)'];
    end
end

% edge extration from right pin model projection under first view frame
index_X = 6+3*num_tibia_edge_first_view+6+3*num_tibia_edge_second_view+...
    6+3*num_edge_left_pin_first_view+3*num_edge_left_pin_second_view;

Alpha_right_pin = X(index_X+1,1);
Beta_right_pin = X(index_X+2,1);
Gamma_right_pin = X(index_X+3,1); 
R_wn_right_pin = RMatrixYPR22(Alpha_right_pin,Beta_right_pin,Gamma_right_pin); % convert to rotation as order of Alpha, Beta and Gamma
t_wn_right_pin = X(index_X+4:index_X+6,1);


ptRightPinModelCamerafirstView = ...
    R_cw_first_view*( R_wn_right_pin*nail_points' + t_wn_right_pin) + t_cw_first_view;

[right_pin_projected_image_first_view, projected_right_pin_points_index_first_view] = ...
    pointCloud_perspective_projection(ptRightPinModelCamerafirstView,K,rows,cols);

[ver_row, ver_col] = find(right_pin_projected_image_first_view == 1);
vertices = [ver_col, ver_row];
holeThreshold = 50;
edge_ratio_top = 0/10;
edge_ratio_bottom = 10/10;
[right_pin_edge_index_first_view, ~] = calculate_projectionContour(vertices,holeThreshold,edge_ratio_top,edge_ratio_bottom);

rightPinModelObs2D_first_view = zeros(0,2);
rightPinModelObs3D_first_view = zeros(0,3);
rightPinModelObs3DCam_first_view = zeros(0,3);

for i = 1:size(right_pin_edge_index_first_view,1)
    v_row = right_pin_edge_index_first_view(i,1);
    u_col = right_pin_edge_index_first_view(i,2);
    index_tmpt = projected_right_pin_points_index_first_view(v_row, u_col);
    if index_tmpt > 0
        rightPinModelObs2D_first_view = [rightPinModelObs2D_first_view;v_row,u_col];
        rightPinModelObs3D_first_view = [rightPinModelObs3D_first_view;nail_points(index_tmpt,:)];
        rightPinModelObs3DCam_first_view = ...
            [rightPinModelObs3DCam_first_view;ptRightPinModelCamerafirstView(:,index_tmpt)'];
    end
end


% edge extration from right pin model projection under second view frame
ptRightPinModelCamerasecondView = ...
    R_cw_second_view*( R_wn_right_pin*nail_points' + t_wn_right_pin) + t_cw_second_view;

[right_pin_projected_image_second_view, projected_right_pin_points_index_second_view] = ...
    pointCloud_perspective_projection(ptRightPinModelCamerasecondView,K,rows,cols);

[ver_row, ver_col] = find(right_pin_projected_image_second_view == 1);
vertices = [ver_col, ver_row];
holeThreshold = 50;
edge_ratio_top = 0/10;
edge_ratio_bottom = 10/10;
[right_pin_edge_index_second_view, ~] = calculate_projectionContour(vertices,holeThreshold,edge_ratio_top,edge_ratio_bottom);

rightPinModelObs2D_second_view = zeros(0,2);
rightPinModelObs3D_second_view = zeros(0,3);
rightPinModelObs3DCam_second_view = zeros(0,3);

for i = 1:size(right_pin_edge_index_second_view,1)
    v_row = right_pin_edge_index_second_view(i,1);
    u_col = right_pin_edge_index_second_view(i,2);
    index_tmpt = projected_right_pin_points_index_second_view(v_row, u_col);
    if index_tmpt > 0
        rightPinModelObs2D_second_view = [rightPinModelObs2D_second_view;v_row,u_col];
        rightPinModelObs3D_second_view = [rightPinModelObs3D_second_view;nail_points(index_tmpt,:)];
        rightPinModelObs3DCam_second_view = [rightPinModelObs3DCam_second_view;ptRightPinModelCamerasecondView(:,index_tmpt)'];
    end
end

num_kneeModleObs2D_first_view = size(kneeModelObs2D_first_view,1);
num_kneeModleObs2D_second_view = size(kneeModelObs2D_second_view,1);
num_leftPinModleObs2D_first_view = size(leftPinModelObs2D_first_view,1);
num_leftPinModleObs2D_second_view = size(leftPinModelObs2D_second_view,1);
num_rightPinModleObs2D_first_view = size(rightPinModelObs2D_first_view,1);
num_rightPinModleObs2D_second_view = size(rightPinModelObs2D_second_view,1);

%% Error and Jacobian reconstruction
num = 5*(num_tibia_edge_first_view + num_tibia_edge_second_view + num_edge_left_pin_first_view + ...
    num_edge_left_pin_second_view + num_edge_right_pin_first_view + num_edge_right_pin_second_view)+...
    num_kneeModleObs2D_first_view + num_kneeModleObs2D_second_view + num_leftPinModleObs2D_first_view + ...
    num_leftPinModleObs2D_second_view + num_rightPinModleObs2D_first_view + num_rightPinModleObs2D_second_view;

Error = zeros(num,1);
Jacobian = zeros(num, 6 + 3*num_tibia_edge_first_view + 6 + 3*num_tibia_edge_second_view+...
    6 + 3*num_edge_left_pin_first_view + 3*num_edge_left_pin_second_view + ...
    6 + 3*num_edge_right_pin_first_view + 3*num_edge_right_pin_second_view); % pose + wolrd points

%% Error from the first view observation of the tibia
% sdf of the tibia edge observation from the first view
edge3dWrd_tibia_frame_first_view = reshape(X(7:6+3*num_tibia_edge_first_view),3,[]); % obtain pts_world from state
dx = TibiaDTXgrid(edge3dWrd_tibia_frame_first_view(1,:), edge3dWrd_tibia_frame_first_view(2,:), edge3dWrd_tibia_frame_first_view(3,:));
dy = TibiaDTYgrid(edge3dWrd_tibia_frame_first_view(1,:), edge3dWrd_tibia_frame_first_view(2,:), edge3dWrd_tibia_frame_first_view(3,:));
dz = TibiaDTZgrid(edge3dWrd_tibia_frame_first_view(1,:), edge3dWrd_tibia_frame_first_view(2,:), edge3dWrd_tibia_frame_first_view(3,:));
Error_dxyz = [dx;dy;dz];% [x1;y1;z1;x2;y2;z2;...;xn;yn;zn],n is the number edge points
Error(1:3*num_tibia_edge_first_view,1) = Error_dxyz(:);

% uv re-projection error from the first observation of tibia
edge3dCam_tibia_frame_first_view = R_cw_first_view*edge3dWrd_tibia_frame_first_view + t_cw_first_view; % transform pts in the world space to the camera space
uv_edge_tibia_frame_first_view = K*(edge3dCam_tibia_frame_first_view(1:3,:) ./ edge3dCam_tibia_frame_first_view(3,:)); % uv estimation
uv_edge_tibia_frame_first_view = uv_edge_tibia_frame_first_view(1:2,:);
Error(3*num_tibia_edge_first_view+1:5*num_tibia_edge_first_view,1) = uv_edge_tibia_frame_first_view(:) - uv_edge_tibia_first_view_gt(:);

%% Error from the second view observation of the tibia
index_X = 6+3*num_tibia_edge_first_view;
index_Error = 5*num_tibia_edge_first_view;

% sdf of the tibia edge observation from the second view
edge3dWrd_tibia_frame_second_view = reshape(X(index_X+7:index_X+6+3*num_tibia_edge_second_view),3,[]); % obtain pts_world from state
dx = TibiaDTXgrid(edge3dWrd_tibia_frame_second_view(1,:), edge3dWrd_tibia_frame_second_view(2,:), edge3dWrd_tibia_frame_second_view(3,:));
dy = TibiaDTYgrid(edge3dWrd_tibia_frame_second_view(1,:), edge3dWrd_tibia_frame_second_view(2,:), edge3dWrd_tibia_frame_second_view(3,:));
dz = TibiaDTZgrid(edge3dWrd_tibia_frame_second_view(1,:), edge3dWrd_tibia_frame_second_view(2,:), edge3dWrd_tibia_frame_second_view(3,:));
Error_dxyz = [dx;dy;dz];% [x1;y1;z1;x2;y2;z2;...;xn;yn;zn],n is the number edge points
Error(index_Error+1:index_Error+3*num_tibia_edge_second_view,1) = Error_dxyz(:);

% uv re-projection error from the second observation of tibia
edge3dCam_tibia_frame_second_view = R_cw_second_view*edge3dWrd_tibia_frame_second_view + t_cw_second_view; % transform pts in the world space to the camera space
uv_edge_tibia_second_second_view = K*(edge3dCam_tibia_frame_second_view(1:3,:) ./ edge3dCam_tibia_frame_second_view(3,:)); % uv estimation
uv_edge_tibia_second_second_view = uv_edge_tibia_second_second_view(1:2,:);
Error(index_Error+3*num_tibia_edge_second_view+1:index_Error+5*num_tibia_edge_second_view,1) = uv_edge_tibia_second_second_view(:) - uv_edge_tibia_second_view_gt(:);

%% Error from the first view observation of the left pin
index_X = 6+3*num_tibia_edge_first_view+6+3*num_tibia_edge_second_view;
index_Error = 5*num_tibia_edge_first_view+5*num_tibia_edge_second_view;

% sdf of the left pin edge observation from first view
edge3dNWrd_left_pin_first_view = reshape(X(index_X+7:index_X+6+3*num_edge_left_pin_first_view),3,[]); % obtain pts_world from state
dx = PinDTXgrid(edge3dNWrd_left_pin_first_view(1,:), edge3dNWrd_left_pin_first_view(2,:), edge3dNWrd_left_pin_first_view(3,:));
dy = PinDTYgrid(edge3dNWrd_left_pin_first_view(1,:), edge3dNWrd_left_pin_first_view(2,:), edge3dNWrd_left_pin_first_view(3,:));
dz = PinDTZgrid(edge3dNWrd_left_pin_first_view(1,:), edge3dNWrd_left_pin_first_view(2,:), edge3dNWrd_left_pin_first_view(3,:));
Error_dxyz = [dx;dy;dz];% [x1;y1;z1;x2;y2;z2;...;xn;yn;zn],n is the number edge points
Error(index_Error+1:index_Error+3*num_edge_left_pin_first_view,1) = Error_dxyz(:);

% uv re-projection error from the first view observation of left pin
edge3dCam_left_pin_first_view = R_cw_first_view*( R_wn_left_pin*edge3dNWrd_left_pin_first_view + t_wn_left_pin) + t_cw_first_view;
uv_edge_left_pin_first_view = K*(edge3dCam_left_pin_first_view(1:3,:) ./ edge3dCam_left_pin_first_view(3,:)); % uv estimation
uv_edge_left_pin_first_view = uv_edge_left_pin_first_view(1:2,:);
Error(index_Error+3*num_edge_left_pin_first_view+1:index_Error+5*num_edge_left_pin_first_view,1) = ...
    uv_edge_left_pin_first_view(:) - uv_edge_left_pin_first_view_gt(:);

%% Error from the second view observation of the left pin
index_X = 6+3*num_tibia_edge_first_view+6+3*num_tibia_edge_second_view+6+3*num_edge_left_pin_first_view;
index_Error = 5*num_tibia_edge_first_view+5*num_tibia_edge_second_view+5*num_edge_left_pin_first_view;

% sdf of the left pin edge observation from second view
edge3dNWrd_left_pin_second_view = reshape(X(index_X+1:index_X+3*num_edge_left_pin_second_view),3,[]); % obtain pts_world from state
dx = PinDTXgrid(edge3dNWrd_left_pin_second_view(1,:), edge3dNWrd_left_pin_second_view(2,:), edge3dNWrd_left_pin_second_view(3,:));
dy = PinDTYgrid(edge3dNWrd_left_pin_second_view(1,:), edge3dNWrd_left_pin_second_view(2,:), edge3dNWrd_left_pin_second_view(3,:));
dz = PinDTZgrid(edge3dNWrd_left_pin_second_view(1,:), edge3dNWrd_left_pin_second_view(2,:), edge3dNWrd_left_pin_second_view(3,:));
Error_dxyz = [dx;dy;dz];% [x1;y1;z1;x2;y2;z2;...;xn;yn;zn],n is the number edge points
Error(index_Error+1:index_Error+3*num_edge_left_pin_second_view,1) = Error_dxyz(:);

% uv re-projection error from the second view observation of left pin
edge3dCam_left_pin_second_view = R_cw_second_view*( R_wn_left_pin*edge3dNWrd_left_pin_second_view + t_wn_left_pin) + t_cw_second_view; % transform pts in the world space to the camera space
uv_edge_left_pin_second_view = K*(edge3dCam_left_pin_second_view(1:3,:) ./ edge3dCam_left_pin_second_view(3,:)); % uv estimation
uv_edge_left_pin_second_view = uv_edge_left_pin_second_view(1:2,:);
Error(index_Error+3*num_edge_left_pin_second_view+1:index_Error+5*num_edge_left_pin_second_view,1) = ...
    uv_edge_left_pin_second_view(:) - uv_edge_left_pin_second_view_gt(:);

%% Error from the first view observation of the right pin
index_X = 6+3*num_tibia_edge_first_view+6+3*num_tibia_edge_second_view+...
    6+3*num_edge_left_pin_first_view+3*num_edge_left_pin_second_view;

index_Error = 5*num_tibia_edge_first_view+5*num_tibia_edge_second_view+...
    5*num_edge_left_pin_first_view+5*num_edge_left_pin_second_view;

% sdf of the right pin edge observation from first view
edge3dNWrd_right_pin_first_view = reshape(X(index_X+7:index_X+6+3*num_edge_right_pin_first_view),3,[]); % obtain pts_world from state
dx = PinDTXgrid(edge3dNWrd_right_pin_first_view(1,:), edge3dNWrd_right_pin_first_view(2,:), edge3dNWrd_right_pin_first_view(3,:));
dy = PinDTYgrid(edge3dNWrd_right_pin_first_view(1,:), edge3dNWrd_right_pin_first_view(2,:), edge3dNWrd_right_pin_first_view(3,:));
dz = PinDTZgrid(edge3dNWrd_right_pin_first_view(1,:), edge3dNWrd_right_pin_first_view(2,:), edge3dNWrd_right_pin_first_view(3,:));
Error_dxyz = [dx;dy;dz];% [x1;y1;z1;x2;y2;z2;...;xn;yn;zn],n is the number edge points
Error(index_Error+1:index_Error+3*num_edge_right_pin_first_view,1) = Error_dxyz(:);

% uv re-projection error from the first view observation of right pin
edge3dCam_right_pin_first_view = R_cw_first_view*( R_wn_right_pin*edge3dNWrd_right_pin_first_view + t_wn_right_pin) + t_cw_first_view; % transform pts in the world space to the camera space
uv_edge_right_pin_first_view = K*(edge3dCam_right_pin_first_view(1:3,:) ./ edge3dCam_right_pin_first_view(3,:)); % uv estimation
uv_edge_right_pin_first_view = uv_edge_right_pin_first_view(1:2,:);
Error(index_Error+3*num_edge_right_pin_first_view+1:index_Error+5*num_edge_right_pin_first_view,1) = ...
    uv_edge_right_pin_first_view(:) - uv_edge_right_pin_first_view_gt(:);

%% Error from the second view observation of the right pin
index_X = 6+3*num_tibia_edge_first_view+6+3*num_tibia_edge_second_view+...
    6+3*num_edge_left_pin_first_view+3*num_edge_left_pin_second_view+6+3*num_edge_right_pin_first_view;
index_Error = 5*num_tibia_edge_first_view+5*num_tibia_edge_second_view+...
    5*num_edge_left_pin_first_view+5*num_edge_left_pin_second_view+5*num_edge_right_pin_first_view;

% sdf of the right pin edge observation from second view
edge3dNWrd_right_pin_second_view = reshape(X(index_X+1:index_X+3*num_edge_right_pin_second_view),3,[]); % obtain pts_world from state
dx = PinDTXgrid(edge3dNWrd_right_pin_second_view(1,:), edge3dNWrd_right_pin_second_view(2,:), edge3dNWrd_right_pin_second_view(3,:));
dy = PinDTYgrid(edge3dNWrd_right_pin_second_view(1,:), edge3dNWrd_right_pin_second_view(2,:), edge3dNWrd_right_pin_second_view(3,:));
dz = PinDTZgrid(edge3dNWrd_right_pin_second_view(1,:), edge3dNWrd_right_pin_second_view(2,:), edge3dNWrd_right_pin_second_view(3,:));
Error_dxyz = [dx;dy;dz];% [x1;y1;z1;x2;y2;z2;...;xn;yn;zn],n is the number edge points
Error(index_Error+1:index_Error+3*num_edge_right_pin_second_view,1) = Error_dxyz(:);

% uv re-projection error from the second view observation of right pin
edge3dCam_right_pin_second_view = R_cw_second_view*( R_wn_right_pin*edge3dNWrd_right_pin_second_view + t_wn_right_pin) + t_cw_second_view; % transform pts in the world space to the camera space
uv_edge_right_pin_second_view = K*(edge3dCam_right_pin_second_view(1:3,:) ./ edge3dCam_right_pin_second_view(3,:)); % uv estimation
uv_edge_right_pin_second_view = uv_edge_right_pin_second_view(1:2,:);
Error(index_Error+3*num_edge_right_pin_second_view+1:index_Error+5*num_edge_right_pin_second_view,1) = ...
    uv_edge_right_pin_second_view(:) - uv_edge_right_pin_second_view_gt(:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Error from the 2D edge tsdf observation
index_Error_start = 5*(num_tibia_edge_first_view+num_tibia_edge_second_view+num_edge_left_pin_first_view+...
    num_edge_left_pin_second_view+num_edge_right_pin_first_view+num_edge_right_pin_second_view);
index_Error = index_Error_start;
Error(index_Error+1:index_Error+num_kneeModleObs2D_first_view) = ...
    KneeEdgeDTgrid{1,1}(kneeModelObs2D_first_view(:,2)',kneeModelObs2D_first_view(:,1)'); %u(x),v(y)

index_Error = index_Error_start + num_kneeModleObs2D_first_view;
Error(index_Error+1:index_Error+num_kneeModleObs2D_second_view) = ...
    KneeEdgeDTgrid{1,2}(kneeModelObs2D_second_view(:,2)',kneeModelObs2D_second_view(:,1)');

index_Error = index_Error_start + num_kneeModleObs2D_first_view + num_kneeModleObs2D_second_view;
Error(index_Error+1:index_Error+num_leftPinModleObs2D_first_view) = ...
    PinEdgeDTgrid{1,1}(leftPinModelObs2D_first_view(:,2)',leftPinModelObs2D_first_view(:,1)');

index_Error = index_Error_start + num_kneeModleObs2D_first_view + ...
    num_kneeModleObs2D_second_view + num_leftPinModleObs2D_first_view;
Error(index_Error+1:index_Error+num_leftPinModleObs2D_second_view) = ...
    PinEdgeDTgrid{1,3}(leftPinModelObs2D_second_view(:,2)',leftPinModelObs2D_second_view(:,1)');

index_Error = index_Error_start + num_kneeModleObs2D_first_view + num_kneeModleObs2D_second_view + ...
    num_leftPinModleObs2D_first_view + num_leftPinModleObs2D_second_view;
Error(index_Error+1:index_Error+num_rightPinModleObs2D_first_view) = ...
    PinEdgeDTgrid{1,2}(rightPinModelObs2D_first_view(:,2)',rightPinModelObs2D_first_view(:,1)');

index_Error = index_Error_start + num_kneeModleObs2D_first_view + num_kneeModleObs2D_second_view + ...
    num_leftPinModleObs2D_first_view + num_leftPinModleObs2D_second_view + num_rightPinModleObs2D_first_view;
Error(index_Error+1:index_Error+num_rightPinModleObs2D_second_view) = ...
    PinEdgeDTgrid{1,4}(rightPinModelObs2D_second_view(:,2)',rightPinModelObs2D_second_view(:,1)');

% compute the sum of error
%Sum_Error = Error'*CovMatrixInv*Error;
Sum_Error = Error'*Error;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Jacobian construction
ID1 = [];
ID2 = [];
Val = [];

%% Jacobian from the first view observation of the tibia
% gradient of dx/dy/dz w.r.t point Pw transformed into the tibia model space
tibia_pts_world = edge3dWrd_tibia_frame_first_view;
Alpha = Alpha_first_view;
Beta = Beta_first_view;
Gamma = Gamma_first_view;

dxdPw = [TibiaGDTXXgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:));...
    TibiaGDTXYgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:));...
    TibiaGDTXZgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:))];

dydPw = [TibiaGDTYXgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:));...
    TibiaGDTYYgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:));...
    TibiaGDTYZgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:))];

dzdPw = [TibiaGDTZXgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:));...
    TibiaGDTZYgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:));...
    TibiaGDTZZgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:))];

% Bundle Adjustment derivation, pixel u,v estimation w.r.t R,t and Pw
% Euler derivation
dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*tibia_pts_world;
dFydABG = [dydA;dydB;dydG]*tibia_pts_world;
dFzdABG = [dzdA;dzdB;dzdG]*tibia_pts_world;

for i = 1:num_tibia_edge_first_view
    % jacobian of dx/dy/dz w.r.t R,t,Pw
    row_index_tmp = (i-1)*3;
    col_index_tmp = 6+(i-1)*3;
    ID1 = [ID1;repelem(row_index_tmp+1,3)';repelem(row_index_tmp+2,3)';repelem(row_index_tmp+3,3)'];
    ID2 = [ID2;col_index_tmp+1;col_index_tmp+2;col_index_tmp+3;...
        col_index_tmp+1;col_index_tmp+2;col_index_tmp+3;col_index_tmp+1;col_index_tmp+2;col_index_tmp+3];
    
    Val = [Val;dxdPw(:,i);dydPw(:,i);dzdPw(:,i)];
    
    % jacobian of u/v w.r.t R,t,Pw
    row_index_uv = 3*num_tibia_edge_first_view+(i-1)*2;
    col_index_uv = 6+(i-1)*3;
    
    Pc = edge3dCam_tibia_frame_first_view(:,i);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc * [dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    duvdPw = duvdPc * R_cw_first_view; % 2-by-3
    
    ID1 = [ID1;repelem(row_index_uv+1,9)';repelem(row_index_uv+2,9)'];
    
    ID2 = [ID2;(1:6)';col_index_uv+1;col_index_uv+2;col_index_uv+3;...
        (1:6)';col_index_uv+1;col_index_uv+2;col_index_uv+3];
    
    Val = [Val;duvdABGt(1,1:end)';duvdPw(1,:)';...
        duvdABGt(2,1:end)';duvdPw(2,:)'];
end

%% Jacobian from the second view observation of the tibia
% gradient of dx/dy/dz w.r.t point Pw transformed into the tibia model space
tibia_pts_world = edge3dWrd_tibia_frame_second_view;
Alpha = Alpha_second_view;
Beta = Beta_second_view;
Gamma = Gamma_second_view;

dxdPw = [TibiaGDTXXgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:));...
    TibiaGDTXYgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:));...
    TibiaGDTXZgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:))];

dydPw = [TibiaGDTYXgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:));...
    TibiaGDTYYgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:));...
    TibiaGDTYZgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:))];

dzdPw = [TibiaGDTZXgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:));...
    TibiaGDTZYgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:));...
    TibiaGDTZZgrid(tibia_pts_world(1,:),tibia_pts_world(2,:),tibia_pts_world(3,:))];

% Bundle Adjustment derivation, pixel u,v estimation w.r.t R,t and Pw
% Euler derivation
dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*tibia_pts_world;
dFydABG = [dydA;dydB;dydG]*tibia_pts_world;
dFzdABG = [dzdA;dzdB;dzdG]*tibia_pts_world;

row_index = 5*num_tibia_edge_first_view;
col_index = 6 + 3*num_tibia_edge_first_view;    
for i = 1:num_tibia_edge_second_view
    % jacobian of dx/dy/dz w.r.t R,t,Pw
    row_index_tmp = row_index+(i-1)*3;
    col_index_tmp = col_index+6+(i-1)*3;
    ID1 = [ID1;repelem(row_index_tmp+1,3)';repelem(row_index_tmp+2,3)';repelem(row_index_tmp+3,3)'];
    ID2 = [ID2;col_index_tmp+1;col_index_tmp+2;col_index_tmp+3;...
        col_index_tmp+1;col_index_tmp+2;col_index_tmp+3;col_index_tmp+1;col_index_tmp+2;col_index_tmp+3];
    
    Val = [Val;dxdPw(:,i);dydPw(:,i);dzdPw(:,i)];
    
    % jacobian of u/v w.r.t R,t,Pw
    row_index_uv = row_index+3*num_tibia_edge_second_view+(i-1)*2;
    col_index_uv = col_index+6+(i-1)*3;
    
    Pc = edge3dCam_tibia_frame_second_view(:,i);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc * [dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    duvdPw = duvdPc * R_cw_second_view; % 2-by-3
    
    ID1 = [ID1;repelem(row_index_uv+1,9)';repelem(row_index_uv+2,9)'];
    
    ID2 = [ID2;(col_index+1:col_index+6)';col_index_uv+1;col_index_uv+2;col_index_uv+3;...
        (col_index+1:col_index+6)';col_index_uv+1;col_index_uv+2;col_index_uv+3];
    
    Val = [Val;duvdABGt(1,1:end)';duvdPw(1,:)';...
        duvdABGt(2,1:end)';duvdPw(2,:)'];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Jacobian from the left pin observation in the first view frame 
% Euler derivation of pin uv observation w.r.t the frame pose of first view
Alpha = Alpha_first_view;
Beta = Beta_first_view;
Gamma = Gamma_first_view;

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*(R_wn_left_pin*edge3dNWrd_left_pin_first_view + t_wn_left_pin);
dFydABG = [dydA;dydB;dydG]*(R_wn_left_pin*edge3dNWrd_left_pin_first_view + t_wn_left_pin);
dFzdABG = [dzdA;dzdB;dzdG]*(R_wn_left_pin*edge3dNWrd_left_pin_first_view + t_wn_left_pin);

% jacobian of u/v w.r.t R,t of X-ray frame of first view
row_index = 5*num_tibia_edge_first_view+5*num_tibia_edge_second_view+3*num_edge_left_pin_first_view;
for i = 1:num_edge_left_pin_first_view    
    row_index_uv = row_index+(i-1)*2;
    
    Pc = edge3dCam_left_pin_first_view(:,i);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc * [dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    
    ID1 = [ID1;repelem(row_index_uv+1,6)';repelem(row_index_uv+2,6)'];
    ID2 = [ID2;(1:6)';(1:6)'];
    Val = [Val;duvdABGt(1,1:end)';duvdABGt(2,1:end)'];
end

% Euler derivation of left pin sdf and uv observation w.r.t the left pin pose
pin_pts_world = edge3dNWrd_left_pin_first_view;
Alpha = Alpha_left_pin;
Beta = Beta_left_pin;
Gamma = Gamma_left_pin;

dxdPw = [PinGDTXXgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTXYgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTXZgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:))];

dydPw = [PinGDTYXgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTYYgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTYZgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:))];

dzdPw = [PinGDTZXgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTZYgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTZZgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:))];

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*pin_pts_world;
dFydABG = [dydA;dydB;dydG]*pin_pts_world;
dFzdABG = [dzdA;dzdB;dzdG]*pin_pts_world;

row_index = 5*num_tibia_edge_first_view+5*num_tibia_edge_second_view;
col_index = 6+3*num_tibia_edge_first_view+6+3*num_tibia_edge_second_view;    
for i = 1:num_edge_left_pin_first_view
    % jacobian of dx/dy/dz w.r.t Pw
    row_index_tmp = row_index+(i-1)*3;
    col_index_tmp = col_index+6+(i-1)*3;
    ID1 = [ID1;repelem(row_index_tmp+1,3)';repelem(row_index_tmp+2,3)';repelem(row_index_tmp+3,3)'];
    ID2 = [ID2;col_index_tmp+1;col_index_tmp+2;col_index_tmp+3;...
        col_index_tmp+1;col_index_tmp+2;col_index_tmp+3;col_index_tmp+1;col_index_tmp+2;col_index_tmp+3];
    
    Val = [Val;dxdPw(:,i);dydPw(:,i);dzdPw(:,i)];
    
    % jacobian of u/v w.r.t R,t,Pw
    row_index_uv = row_index+3*num_edge_left_pin_first_view+(i-1)*2;
    col_index_uv = col_index+6+(i-1)*3;
    
    Pc = edge3dCam_left_pin_first_view(:,i);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc*R_cw_first_view*[dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    duvdPw = duvdPc*R_cw_first_view*R_wn_left_pin; % 2-by-3
    
    ID1 = [ID1;repelem(row_index_uv+1,9)';repelem(row_index_uv+2,9)'];
    
    ID2 = [ID2;(col_index+1:col_index+6)';col_index_uv+1;col_index_uv+2;col_index_uv+3;...
        (col_index+1:col_index+6)';col_index_uv+1;col_index_uv+2;col_index_uv+3];
    
    Val = [Val;duvdABGt(1,1:end)';duvdPw(1,:)';...
        duvdABGt(2,1:end)';duvdPw(2,:)'];
end


%% Jacobian from the left pin observation in the second view frame 
% Euler derivation of left pin uv observation w.r.t the frame pose of second view
Alpha = Alpha_second_view;
Beta = Beta_second_view;
Gamma = Gamma_second_view;

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*(R_wn_left_pin*edge3dNWrd_left_pin_second_view + t_wn_left_pin);
dFydABG = [dydA;dydB;dydG]*(R_wn_left_pin*edge3dNWrd_left_pin_second_view + t_wn_left_pin);
dFzdABG = [dzdA;dzdB;dzdG]*(R_wn_left_pin*edge3dNWrd_left_pin_second_view + t_wn_left_pin);

% jacobian of u/v w.r.t R,t of X-ray frame of second view
row_index = 5*num_tibia_edge_first_view+5*num_tibia_edge_second_view+...
    5*num_edge_left_pin_first_view+3*num_edge_left_pin_second_view;
col_index = 6+3*num_tibia_edge_first_view;
for i = 1:num_edge_left_pin_second_view    
    row_index_uv = row_index+(i-1)*2;
    
    Pc = edge3dCam_left_pin_second_view(:,i);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc * [dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    
    ID1 = [ID1;repelem(row_index_uv+1,6)';repelem(row_index_uv+2,6)'];
    ID2 = [ID2;(col_index+1:col_index+6)';(col_index+1:col_index+6)'];
    Val = [Val;duvdABGt(1,1:end)';duvdABGt(2,1:end)'];
end

% Euler derivation of left pin sdf and uv observation w.r.t the left pin pose
pin_pts_world = edge3dNWrd_left_pin_second_view;
Alpha = Alpha_left_pin;
Beta = Beta_left_pin;
Gamma = Gamma_left_pin;

dxdPw = [PinGDTXXgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTXYgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTXZgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:))];

dydPw = [PinGDTYXgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTYYgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTYZgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:))];

dzdPw = [PinGDTZXgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTZYgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTZZgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:))];

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*pin_pts_world;
dFydABG = [dydA;dydB;dydG]*pin_pts_world;
dFzdABG = [dzdA;dzdB;dzdG]*pin_pts_world;

row_index = 5*num_tibia_edge_first_view+5*num_tibia_edge_second_view+...
    5*num_edge_left_pin_first_view;
col_index = 6+3*num_tibia_edge_first_view+6+3*num_tibia_edge_second_view;    
for i = 1:num_edge_left_pin_second_view
    % jacobian of dx/dy/dz w.r.t Pw
    row_index_tmp = row_index+(i-1)*3;
    col_index_tmp = col_index+6+3*num_edge_left_pin_first_view+(i-1)*3;
    ID1 = [ID1;repelem(row_index_tmp+1,3)';repelem(row_index_tmp+2,3)';repelem(row_index_tmp+3,3)'];
    ID2 = [ID2;col_index_tmp+1;col_index_tmp+2;col_index_tmp+3;...
        col_index_tmp+1;col_index_tmp+2;col_index_tmp+3;col_index_tmp+1;col_index_tmp+2;col_index_tmp+3];
    
    Val = [Val;dxdPw(:,i);dydPw(:,i);dzdPw(:,i)];
    
    % jacobian of u/v w.r.t R,t,Pw
    row_index_uv = row_index+3*num_edge_left_pin_second_view+(i-1)*2;
    col_index_uv = col_index+6+3*num_edge_left_pin_first_view+(i-1)*3;
    
    Pc = edge3dCam_left_pin_second_view(:,i);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc*R_cw_second_view*[dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    duvdPw = duvdPc*R_cw_second_view*R_wn_left_pin; % 2-by-3
    
    ID1 = [ID1;repelem(row_index_uv+1,9)';repelem(row_index_uv+2,9)'];
    
    ID2 = [ID2;(col_index+1:col_index+6)';col_index_uv+1;col_index_uv+2;col_index_uv+3;...
        (col_index+1:col_index+6)';col_index_uv+1;col_index_uv+2;col_index_uv+3];
    
    Val = [Val;duvdABGt(1,1:end)';duvdPw(1,:)';...
        duvdABGt(2,1:end)';duvdPw(2,:)'];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Jacobian from the right pin observation in the first view frame 
% Euler derivation of pin uv observation w.r.t the frame pose of first view
Alpha = Alpha_first_view;
Beta = Beta_first_view;
Gamma = Gamma_first_view;

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*(R_wn_right_pin*edge3dNWrd_right_pin_first_view + t_wn_right_pin);
dFydABG = [dydA;dydB;dydG]*(R_wn_right_pin*edge3dNWrd_right_pin_first_view + t_wn_right_pin);
dFzdABG = [dzdA;dzdB;dzdG]*(R_wn_right_pin*edge3dNWrd_right_pin_first_view + t_wn_right_pin);

% jacobian of u/v w.r.t R,t of X-ray frame of first view
row_index = 5*num_tibia_edge_first_view+5*num_tibia_edge_second_view+5*num_edge_left_pin_first_view+...
    5*num_edge_left_pin_second_view+3*num_edge_right_pin_first_view;
for i = 1:num_edge_right_pin_first_view    
    row_index_uv = row_index+(i-1)*2;
    
    Pc = edge3dCam_right_pin_first_view(:,i);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc * [dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    
    ID1 = [ID1;repelem(row_index_uv+1,6)';repelem(row_index_uv+2,6)'];
    ID2 = [ID2;(1:6)';(1:6)'];
    Val = [Val;duvdABGt(1,1:end)';duvdABGt(2,1:end)'];
end

% Euler derivation of right pin sdf and uv observation w.r.t the right pin pose
pin_pts_world = edge3dNWrd_right_pin_first_view;
Alpha = Alpha_right_pin;
Beta = Beta_right_pin;
Gamma = Gamma_right_pin;

dxdPw = [PinGDTXXgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTXYgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTXZgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:))];

dydPw = [PinGDTYXgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTYYgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTYZgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:))];

dzdPw = [PinGDTZXgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTZYgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTZZgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:))];

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*pin_pts_world;
dFydABG = [dydA;dydB;dydG]*pin_pts_world;
dFzdABG = [dzdA;dzdB;dzdG]*pin_pts_world;

row_index = 5*num_tibia_edge_first_view+5*num_tibia_edge_second_view+...
    5*num_edge_left_pin_first_view+5*num_edge_left_pin_second_view; 
col_index = 6+3*num_tibia_edge_first_view+6+3*num_tibia_edge_second_view+...
    6+3*num_edge_left_pin_first_view+3*num_edge_left_pin_second_view;

for i = 1:num_edge_right_pin_first_view
    % jacobian of dx/dy/dz w.r.t Pw
    row_index_tmp = row_index+(i-1)*3;
    col_index_tmp = col_index+6+(i-1)*3;
    ID1 = [ID1;repelem(row_index_tmp+1,3)';repelem(row_index_tmp+2,3)';repelem(row_index_tmp+3,3)'];
    ID2 = [ID2;col_index_tmp+1;col_index_tmp+2;col_index_tmp+3;...
        col_index_tmp+1;col_index_tmp+2;col_index_tmp+3;col_index_tmp+1;col_index_tmp+2;col_index_tmp+3];
    
    Val = [Val;dxdPw(:,i);dydPw(:,i);dzdPw(:,i)];
    
    % jacobian of u/v w.r.t R,t,Pw
    row_index_uv = row_index+3*num_edge_right_pin_first_view+(i-1)*2;
    col_index_uv = col_index+6+(i-1)*3;
    
    Pc = edge3dCam_right_pin_first_view(:,i);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc*R_cw_first_view*[dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    duvdPw = duvdPc*R_cw_first_view*R_wn_right_pin; % 2-by-3
    
    ID1 = [ID1;repelem(row_index_uv+1,9)';repelem(row_index_uv+2,9)'];
    
    ID2 = [ID2;(col_index+1:col_index+6)';col_index_uv+1;col_index_uv+2;col_index_uv+3;...
        (col_index+1:col_index+6)';col_index_uv+1;col_index_uv+2;col_index_uv+3];
    
    Val = [Val;duvdABGt(1,1:end)';duvdPw(1,:)';...
        duvdABGt(2,1:end)';duvdPw(2,:)'];
end


%% Jacobian from the right pin observation in the second view frame 
% Euler derivation of left pin uv observation w.r.t the frame pose of second view
Alpha = Alpha_second_view;
Beta = Beta_second_view;
Gamma = Gamma_second_view;

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*(R_wn_right_pin*edge3dNWrd_right_pin_second_view + t_wn_right_pin);
dFydABG = [dydA;dydB;dydG]*(R_wn_right_pin*edge3dNWrd_right_pin_second_view + t_wn_right_pin);
dFzdABG = [dzdA;dzdB;dzdG]*(R_wn_right_pin*edge3dNWrd_right_pin_second_view + t_wn_right_pin);

% jacobian of u/v w.r.t R,t of X-ray frame of second view
row_index = 5*num_tibia_edge_first_view+5*num_tibia_edge_second_view+5*num_edge_left_pin_first_view+...
    5*num_edge_left_pin_second_view+5*num_edge_right_pin_first_view+3*num_edge_right_pin_second_view;
col_index = 6+3*num_tibia_edge_first_view;
for i = 1:num_edge_right_pin_second_view    
    row_index_uv = row_index+(i-1)*2;
    
    Pc = edge3dCam_right_pin_second_view(:,i);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc * [dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    
    ID1 = [ID1;repelem(row_index_uv+1,6)';repelem(row_index_uv+2,6)'];
    ID2 = [ID2;(col_index+1:col_index+6)';(col_index+1:col_index+6)'];
    Val = [Val;duvdABGt(1,1:end)';duvdABGt(2,1:end)'];
end

% Euler derivation of left pin sdf and uv observation w.r.t the left pin pose
pin_pts_world = edge3dNWrd_right_pin_second_view;
Alpha = Alpha_right_pin;
Beta = Beta_right_pin;
Gamma = Gamma_right_pin;

dxdPw = [PinGDTXXgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTXYgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTXZgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:))];

dydPw = [PinGDTYXgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTYYgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTYZgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:))];

dzdPw = [PinGDTZXgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTZYgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:));...
    PinGDTZZgrid(pin_pts_world(1,:),pin_pts_world(2,:),pin_pts_world(3,:))];

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*pin_pts_world;
dFydABG = [dydA;dydB;dydG]*pin_pts_world;
dFzdABG = [dzdA;dzdB;dzdG]*pin_pts_world;

row_index = 5*num_tibia_edge_first_view+5*num_tibia_edge_second_view+5*num_edge_left_pin_first_view+...
    5*num_edge_left_pin_second_view+5*num_edge_right_pin_first_view;
col_index = 6+3*num_tibia_edge_first_view+6+3*num_tibia_edge_second_view+...
    6+3*num_edge_left_pin_first_view+3*num_edge_left_pin_second_view;

for i = 1:num_edge_right_pin_second_view
    % jacobian of dx/dy/dz w.r.t Pw
    row_index_tmp = row_index+(i-1)*3;
    col_index_tmp = col_index+6+3*num_edge_right_pin_first_view+(i-1)*3;
    ID1 = [ID1;repelem(row_index_tmp+1,3)';repelem(row_index_tmp+2,3)';repelem(row_index_tmp+3,3)'];
    ID2 = [ID2;col_index_tmp+1;col_index_tmp+2;col_index_tmp+3;...
        col_index_tmp+1;col_index_tmp+2;col_index_tmp+3;col_index_tmp+1;col_index_tmp+2;col_index_tmp+3];
    
    Val = [Val;dxdPw(:,i);dydPw(:,i);dzdPw(:,i)];
    
    % jacobian of u/v w.r.t R,t,Pw
    row_index_uv = row_index+3*num_edge_right_pin_second_view+(i-1)*2;
    col_index_uv = col_index+6+3*num_edge_right_pin_first_view+(i-1)*3;
    
    Pc = edge3dCam_right_pin_second_view(:,i);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc*R_cw_second_view*[dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    duvdPw = duvdPc*R_cw_second_view*R_wn_right_pin; % 2-by-3
    
    ID1 = [ID1;repelem(row_index_uv+1,9)';repelem(row_index_uv+2,9)'];
    
    ID2 = [ID2;(col_index+1:col_index+6)';col_index_uv+1;col_index_uv+2;col_index_uv+3;...
        (col_index+1:col_index+6)';col_index_uv+1;col_index_uv+2;col_index_uv+3];
    
    Val = [Val;duvdABGt(1,1:end)';duvdPw(1,:)';...
        duvdABGt(2,1:end)';duvdPw(2,:)'];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2D sdf: Jacobian from the knee model edge observation from the first view
Alpha = Alpha_first_view;
Beta = Beta_first_view;
Gamma = Gamma_first_view;

dfduv = [KneeEdgeGXgrid{1,1}(kneeModelObs2D_first_view(:,2)',kneeModelObs2D_first_view(:,1)');...
    KneeEdgeGYgrid{1,1}(kneeModelObs2D_first_view(:,2)',kneeModelObs2D_first_view(:,1)')];

% Bundle Adjustment derivation, pixel u,v estimation w.r.t R,t and Pw
% Euler derivation
dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*kneeModelObs3D_first_view';
dFydABG = [dydA;dydB;dydG]*kneeModelObs3D_first_view';
dFzdABG = [dzdA;dzdB;dzdG]*kneeModelObs3D_first_view';

row_index_start = 5*(num_tibia_edge_first_view+num_tibia_edge_second_view+num_edge_left_pin_first_view+...
    num_edge_left_pin_second_view+num_edge_right_pin_first_view+num_edge_right_pin_second_view);
row_index = row_index_start;

for i = 1:num_kneeModleObs2D_first_view
    % jacobian of sdf w.r.t R,t
    Pc = kneeModelObs3DCam_first_view(i,:);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc * [dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    
    dfdABGt = dfduv(:,i)' * duvdABGt;
    
    row_index_tmp = row_index + i;
    ID1 = [ID1;repelem(row_index_tmp,6)'];
    ID2 = [ID2;(1:6)'];
    
    Val = [Val;dfdABGt(1,1:end)'];
end

%% Jacobian from the knee model edge observation from the second view
Alpha = Alpha_second_view;
Beta = Beta_second_view;
Gamma = Gamma_second_view;

dfduv = [KneeEdgeGXgrid{1,2}(kneeModelObs2D_second_view(:,2)',kneeModelObs2D_second_view(:,1)');...
    KneeEdgeGYgrid{1,2}(kneeModelObs2D_second_view(:,2)',kneeModelObs2D_second_view(:,1)')];

% Bundle Adjustment derivation, pixel u,v estimation w.r.t R,t and Pw
% Euler derivation
dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*kneeModelObs3D_second_view';
dFydABG = [dydA;dydB;dydG]*kneeModelObs3D_second_view';
dFzdABG = [dzdA;dzdB;dzdG]*kneeModelObs3D_second_view';


row_index = row_index_start + num_kneeModleObs2D_first_view;
col_index = 6+3*num_tibia_edge_first_view;    
for i = 1:num_kneeModleObs2D_second_view
    % jacobian of sdf w.r.t R,t
    Pc = kneeModelObs3DCam_second_view(i,:);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc * [dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    
    dfdABGt = dfduv(:,i)' * duvdABGt;
    
    row_index_tmp = row_index + i;
    ID1 = [ID1;repelem(row_index_tmp,6)'];
    ID2 = [ID2;(col_index+1:col_index+6)'];
    
    Val = [Val;dfdABGt(1,1:end)'];
end

%% Jacobian from the left pin model edge observation from the first view
% left pin model projection edge w.r.t the pose of first view frame 
Alpha = Alpha_first_view;
Beta = Beta_first_view;
Gamma = Gamma_first_view;

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*(R_wn_left_pin*leftPinModelObs3D_first_view' + t_wn_left_pin);
dFydABG = [dydA;dydB;dydG]*(R_wn_left_pin*leftPinModelObs3D_first_view' + t_wn_left_pin);
dFzdABG = [dzdA;dzdB;dzdG]*(R_wn_left_pin*leftPinModelObs3D_first_view' + t_wn_left_pin);

dfduv = [PinEdgeGXgrid{1,1}(leftPinModelObs2D_first_view(:,2)',leftPinModelObs2D_first_view(:,1)');...
    PinEdgeGYgrid{1,1}(leftPinModelObs2D_first_view(:,2)',leftPinModelObs2D_first_view(:,1)')];

row_index = row_index_start + num_kneeModleObs2D_first_view + num_kneeModleObs2D_second_view;
for i = 1:num_leftPinModleObs2D_first_view
    % jacobian of sdf w.r.t R,t
    Pc = leftPinModelObs3DCam_first_view(i,:);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc * [dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    
    dfdABGt = dfduv(:,i)' * duvdABGt;
    
    row_index_tmp = row_index + i;
    ID1 = [ID1;repelem(row_index_tmp,6)'];
    ID2 = [ID2;(1:6)'];
    
    Val = [Val;dfdABGt(1,1:end)'];
end

% left pin model projection edge (under the first view frame) w.r.t its pose 
Alpha = Alpha_left_pin;
Beta = Beta_left_pin;
Gamma = Gamma_left_pin;

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*leftPinModelObs3D_first_view';
dFydABG = [dydA;dydB;dydG]*leftPinModelObs3D_first_view';
dFzdABG = [dzdA;dzdB;dzdG]*leftPinModelObs3D_first_view';

row_index = row_index_start + num_kneeModleObs2D_first_view + ...
    num_kneeModleObs2D_second_view;
col_index = 6+3*num_tibia_edge_first_view+6+3*num_tibia_edge_second_view;

for i = 1:num_leftPinModleObs2D_first_view
    % jacobian of sdf w.r.t Rn,tn
    row_index_tmp = row_index+i;
    ID1 = [ID1;repelem(row_index_tmp,6)'];
    ID2 = [ID2;(col_index+1:col_index+6)'];
    
    Pc = leftPinModelObs3DCam_first_view(i,:);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
   
    duvdABGt = duvdPc*R_cw_first_view*[dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    dfdABGt = dfduv(:,i)' * duvdABGt;
    
    Val = [Val;dfdABGt(1,1:end)'];
end

%% Jacobian from the left pin model edge observation from the second view
% left pin model projection edge w.r.t the pose of second view frame 
Alpha = Alpha_second_view;
Beta = Beta_second_view;
Gamma = Gamma_second_view;

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*(R_wn_left_pin*leftPinModelObs3D_second_view' + t_wn_left_pin);
dFydABG = [dydA;dydB;dydG]*(R_wn_left_pin*leftPinModelObs3D_second_view' + t_wn_left_pin);
dFzdABG = [dzdA;dzdB;dzdG]*(R_wn_left_pin*leftPinModelObs3D_second_view' + t_wn_left_pin);

dfduv = [PinEdgeGXgrid{1,3}(leftPinModelObs2D_second_view(:,2)',leftPinModelObs2D_second_view(:,1)');...
    PinEdgeGYgrid{1,3}(leftPinModelObs2D_second_view(:,2)',leftPinModelObs2D_second_view(:,1)')];

row_index = row_index_start + num_kneeModleObs2D_first_view + ...
    num_kneeModleObs2D_second_view + num_leftPinModleObs2D_first_view;
col_index = 6+3*num_tibia_edge_first_view;

for i = 1:num_leftPinModleObs2D_second_view
    % jacobian of sdf w.r.t R,t
    Pc = leftPinModelObs3DCam_second_view(i,:);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc * [dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    
    dfdABGt = dfduv(:,i)' * duvdABGt;
    
    row_index_tmp = row_index + i;
    ID1 = [ID1;repelem(row_index_tmp,6)'];
    ID2 = [ID2;(col_index+1:col_index+6)'];
    
    Val = [Val;dfdABGt(1,1:end)'];
end

% left pin model projection edge (under the second view frame) w.r.t its pose 
Alpha = Alpha_left_pin;
Beta = Beta_left_pin;
Gamma = Gamma_left_pin;

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*leftPinModelObs3D_second_view';
dFydABG = [dydA;dydB;dydG]*leftPinModelObs3D_second_view';
dFzdABG = [dzdA;dzdB;dzdG]*leftPinModelObs3D_second_view';

row_index = row_index_start + num_kneeModleObs2D_first_view + ...
    num_kneeModleObs2D_second_view + num_leftPinModleObs2D_first_view;
col_index = 6+3*num_tibia_edge_first_view+6+3*num_tibia_edge_second_view;

for i = 1:num_leftPinModleObs2D_second_view
    % jacobian of sdf w.r.t Rn,tn
    row_index_tmp = row_index+i;
    ID1 = [ID1;repelem(row_index_tmp,6)'];
    ID2 = [ID2;(col_index+1:col_index+6)'];
    
    Pc = leftPinModelObs3DCam_second_view(i,:);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
   
    duvdABGt = duvdPc*R_cw_second_view*[dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    dfdABGt = dfduv(:,i)' * duvdABGt;
    
    Val = [Val;dfdABGt(1,1:end)'];
end

%% Jacobian from the right pin model edge observation from the first view
% right pin model projection edge w.r.t the pose of first view frame 
Alpha = Alpha_first_view;
Beta = Beta_first_view;
Gamma = Gamma_first_view;

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*(R_wn_right_pin*rightPinModelObs3D_first_view' + t_wn_right_pin);
dFydABG = [dydA;dydB;dydG]*(R_wn_right_pin*rightPinModelObs3D_first_view' + t_wn_right_pin);
dFzdABG = [dzdA;dzdB;dzdG]*(R_wn_right_pin*rightPinModelObs3D_first_view' + t_wn_right_pin);

dfduv = [PinEdgeGXgrid{1,2}(rightPinModelObs2D_first_view(:,2)',rightPinModelObs2D_first_view(:,1)');...
    PinEdgeGYgrid{1,2}(rightPinModelObs2D_first_view(:,2)',rightPinModelObs2D_first_view(:,1)')];

row_index = row_index_start + num_kneeModleObs2D_first_view + num_kneeModleObs2D_second_view + ...
    num_leftPinModleObs2D_first_view + num_leftPinModleObs2D_second_view;

for i = 1:num_rightPinModleObs2D_first_view
    % jacobian of sdf w.r.t R,t
    Pc = rightPinModelObs3DCam_first_view(i,:);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc * [dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    
    dfdABGt = dfduv(:,i)' * duvdABGt;
    
    row_index_tmp = row_index + i;
    ID1 = [ID1;repelem(row_index_tmp,6)'];
    ID2 = [ID2;(1:6)'];
    
    Val = [Val;dfdABGt(1,1:end)'];
end

% right pin model projection edge (under the first view frame) w.r.t its pose
Alpha = Alpha_right_pin;
Beta = Beta_right_pin;
Gamma = Gamma_right_pin;

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*rightPinModelObs3D_first_view';
dFydABG = [dydA;dydB;dydG]*rightPinModelObs3D_first_view';
dFzdABG = [dzdA;dzdB;dzdG]*rightPinModelObs3D_first_view';

row_index = row_index_start + num_kneeModleObs2D_first_view + num_kneeModleObs2D_second_view + ...
    num_leftPinModleObs2D_first_view + num_leftPinModleObs2D_second_view;

col_index = 6+3*num_tibia_edge_first_view+6+3*num_tibia_edge_second_view+...
    6+3*num_edge_left_pin_first_view+3*num_edge_left_pin_second_view;

for i = 1:num_rightPinModleObs2D_first_view
    % jacobian of sdf w.r.t Rn,tn
    row_index_tmp = row_index+i;
    ID1 = [ID1;repelem(row_index_tmp,6)'];
    ID2 = [ID2;(col_index+1:col_index+6)'];
    
    Pc = rightPinModelObs3DCam_first_view(i,:);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
   
    duvdABGt = duvdPc*R_cw_first_view*[dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    dfdABGt = dfduv(:,i)' * duvdABGt;
    
    Val = [Val;dfdABGt(1,1:end)'];
end


%% Jacobian from the right pin model edge observation from the second view
% right pin model projection edge w.r.t the pose of second view frame 
Alpha = Alpha_second_view;
Beta = Beta_second_view;
Gamma = Gamma_second_view;

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*(R_wn_right_pin*rightPinModelObs3D_second_view' + t_wn_right_pin);
dFydABG = [dydA;dydB;dydG]*(R_wn_right_pin*rightPinModelObs3D_second_view' + t_wn_right_pin);
dFzdABG = [dzdA;dzdB;dzdG]*(R_wn_right_pin*rightPinModelObs3D_second_view' + t_wn_right_pin);

dfduv = [PinEdgeGXgrid{1,4}(rightPinModelObs2D_second_view(:,2)',rightPinModelObs2D_second_view(:,1)');...
    PinEdgeGYgrid{1,4}(rightPinModelObs2D_second_view(:,2)',rightPinModelObs2D_second_view(:,1)')];

row_index = row_index_start + num_kneeModleObs2D_first_view + num_kneeModleObs2D_second_view + ...
    num_leftPinModleObs2D_first_view + num_leftPinModleObs2D_second_view + num_rightPinModleObs2D_first_view;

col_index = 6+3*num_tibia_edge_first_view;

for i = 1:num_rightPinModleObs2D_second_view
    % jacobian of sdf w.r.t R,t
    Pc = rightPinModelObs3DCam_second_view(i,:);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
    duvdABGt = duvdPc * [dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    
    dfdABGt = dfduv(:,i)' * duvdABGt;
    
    row_index_tmp = row_index + i;
    ID1 = [ID1;repelem(row_index_tmp,6)'];
    ID2 = [ID2;(col_index+1:col_index+6)'];
    
    Val = [Val;dfdABGt(1,1:end)'];
end

% right pin model projection edge (under the second view frame) w.r.t its pose
Alpha = Alpha_right_pin;
Beta = Beta_right_pin;
Gamma = Gamma_right_pin;

dxdA = [-cos(Beta)*sin(Alpha), cos(Beta)*cos(Alpha),0];
dxdB = [-sin(Beta)*cos(Alpha), -sin(Beta)*sin(Alpha), -cos(Beta)];
dxdG = [0,0,0];

dydA = [-sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha),...
    sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha), 0];
dydB = [sin(Gamma)*cos(Beta)*cos(Alpha), sin(Gamma)*cos(Beta)*sin(Alpha), -sin(Gamma)*sin(Beta)];
dydG = [cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), ...
    cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha), cos(Gamma)*cos(Beta)];

dzdA = [-cos(Gamma)*sin(Beta)*sin(Alpha)+sin(Gamma)*cos(Alpha), ...
    cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha), 0];
dzdB = [cos(Gamma)*cos(Beta)*cos(Alpha), cos(Gamma)*cos(Beta)*sin(Alpha), -cos(Gamma)*sin(Beta)];
dzdG = [-sin(Gamma)*sin(Beta)*cos(Alpha)+cos(Gamma)*sin(Alpha),...
    -sin(Gamma)*sin(Beta)*sin(Alpha)-cos(Gamma)*cos(Alpha), -sin(Gamma)*cos(Beta)];

dFxdABG = [dxdA;dxdB;dxdG]*rightPinModelObs3D_second_view';
dFydABG = [dydA;dydB;dydG]*rightPinModelObs3D_second_view';
dFzdABG = [dzdA;dzdB;dzdG]*rightPinModelObs3D_second_view';

row_index = row_index_start + num_kneeModleObs2D_first_view + num_kneeModleObs2D_second_view + ...
    num_leftPinModleObs2D_first_view + num_leftPinModleObs2D_second_view + num_rightPinModleObs2D_first_view;

col_index = 6+3*num_tibia_edge_first_view+6+3*num_tibia_edge_second_view+...
    6+3*num_edge_left_pin_first_view+3*num_edge_left_pin_second_view;

for i = 1:num_rightPinModleObs2D_second_view
    % jacobian of sdf w.r.t Rn,tn
    row_index_tmp = row_index+i;
    ID1 = [ID1;repelem(row_index_tmp,6)'];
    ID2 = [ID2;(col_index+1:col_index+6)'];
    
    Pc = rightPinModelObs3DCam_second_view(i,:);
    duvdPc = [K(1,1)/Pc(3), 0, -K(1,1)*Pc(1)/(Pc(3)^2);...
        0, K(2,2)/Pc(3), -K(2,2)*Pc(2)/(Pc(3)^2)];
   
    duvdABGt = duvdPc*R_cw_second_view*[dFxdABG(:,i)',1,0,0;dFydABG(:,i)',0,1,0;dFzdABG(:,i)',0,0,1]; % 2-by-6
    dfdABGt = dfduv(:,i)' * duvdABGt;
    
    Val = [Val;dfdABGt(1,1:end)'];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Val = double(Val);
Jacobian = sparse(ID1,ID2,Val);

end