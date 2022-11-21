function [time] = getTime(struct,i)
% getTime gets the time of each message with t=0 at the first message
    Sec_init = cast(struct{1}.Header.Stamp.Sec,"double");
    Nsec_init = cast(struct{1}.Header.Stamp.Nsec,"double");
    time_init = Sec_init + Nsec_init*(10^-9);
    Sec = cast(struct{i}.Header.Stamp.Sec,"double");
    Nsec = cast(struct{i}.Header.Stamp.Nsec,"double");
    time = Sec + Nsec*(10^-9); %- time_init;
end