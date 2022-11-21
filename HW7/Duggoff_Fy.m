function [Fy] = Duggoff_Fy(alpha,C_alpha,Fz,mu)
%DUGGOFF_FY Summary of this function goes here
%   Detailed explanation goes here

f_lamb = f_lambda(alpha,C_alpha,Fz,mu);

Fy = C_alpha * tan(alpha)*f_lamb;
end

