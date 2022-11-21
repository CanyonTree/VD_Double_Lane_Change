function [data_cut,time_cut] = cut_data(data,time,t0,tfinal)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

data_cut = [];
time_cut = [];

for i = 1:length(data)
    if((time(i) > t0) && (time(i) < tfinal))
        data_cut(end+1) = data(i);
        time_cut(end+1) = time(i);
    end
end




end

