function [Xstate] = FuncUpdate(Xstate,Delta)
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
Xstate = Xstate+Delta;
%Xstate(1:6,1) = Xstate(1:6,1)+Delta; % only update pose
%Xstate(7:end,1) = Xstate(7:end,1)+Delta; % only update point in world space
end
