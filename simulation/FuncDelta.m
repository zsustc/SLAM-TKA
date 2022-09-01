
function [Delta,Sum_Delta] = FuncDelta(Jacobian,Error,CovMatrixInv)
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
%Jacobian
%Info = Jacobian'*CovMatrixInv*Jacobian;
Info = Jacobian'*Jacobian;
%check_Info=eig(Info)
%E = -Jacobian'*CovMatrixInv*Error;
E = -Jacobian'*Error;
%pause

Delta = Info\E;
Sum_Delta = Delta'*Delta;

end