function [f_lamb] = f_lambda(alpha,C_alpha,Fz,mu)
%F_LAMBDA Summary of this function goes here
%   Detailed explanation goes here

lamb = (mu*Fz)/(2*C_alpha*abs(tan(alpha)));

if lamb < 1
    f_lamb = (2-lamb)*lamb;
else
    f_lamb = 1;
end


end

