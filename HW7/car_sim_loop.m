function [sim_output] = car_sim_loop(data,car_params)
%CAR_SIM_LOOP Summary of this function goes here
%   Detailed explanation goes here

a = car_params.a;
b = car_params.b;

B = data(1,5);
r = data(1,4);


sim_output(1).time = data(1);
sim_output(1).r = r;
sim_output(1).rd = 0;
sim_output(1).rdd = [];
sim_output(1).B = B;
sim_output(1).Bd = 0;
sim_output(1).Bdd = [];
sim_output(1).Fyf = 0;
sim_output(1).Fyr = 0;

V = data(1,2);
Vx = V * cos(B);
Vy = V * sin(B);
sim_output(1).alphaF = (Vy + a*r)/Vx - data(1,3);
sim_output(1).alphaR = (Vy - b*r)/Vx;


for i = 2:length(data)
    
    time = data(i,1);

    sim_input(i).dt = data(i,1) - data(i-1,1); 
    sim_input(i).V = data(i,2);
    sim_input(i).del = data(i,3);
    sim_input(i).deld = (data(i,3) - data(i-1,3))/sim_input(i).dt;
    sim_input(i).r = sim_output(i-1).r;
    sim_input(i).rd = sim_output(i-1).rd;
    sim_input(i).B = sim_output(i-1).B;
    sim_input(i).Bd = sim_output(i-1).Bd;
    sim_input(i).Fyf = sim_output(i-1).Fyf;
    sim_input(i).Fyr = sim_output(i-1).Fyr;


    if isempty(sim_output(i-1).r) == 1
        sim_input(i).r = data(i-1,4);
        sim_input(i).rd = 0;
        sim_input(i).B = data(i-1,5);
        sim_input(i).Bd = 0;
    end


    if sim_input(i).V > car_params.min_V
      [rdd,rd,r,Bdd,Bd,B,Keff,Beff,rTF,BTF,kus,V_crit,Fyf,Fyr] = car_sim(sim_input(i),car_params);
    else
        rdd = [];
        rd = [];
        r = [];
        Bdd = [];
        Bd = [];
        B = [];
        Keff = [];
        Beff = [];
        rTF = [];
        BTF = [];
        kus = [];
        V_crit = [];
        time = [];
    end
    
    
    
    
    sim_output(i).r = r;
    sim_output(i).rd = rd;
    sim_output(i).rdd = rdd;
    sim_output(i).B = B;
    sim_output(i).Bd = Bd;
    sim_output(i).Bdd = Bdd;
    sim_output(i).Fyf = Fyf;
    sim_output(i).Fyr = Fyr;

 
    
    sim_output(i).dt = sim_input(i).dt;
    sim_output(i).V = sim_input(i).V;
    sim_output(i).del = sim_input(i).del;
    sim_output(i).deld = sim_input(i).deld;
    sim_output(i).time = time;
    sim_output(i).Keff = Keff;
    sim_output(i).Beff = Beff;
    sim_output(i).rTF = rTF;
    sim_output(i).BTF = BTF;

    Vx = sim_input(i).V * cos(B);
    Vy = sim_input(i).V * sin(B);
    
    sim_output(i).alphaF = (Vy + a*r)/Vx - sim_output(i).del;
    sim_output(i).alphaR = (Vy - b*r)/Vx;
    sim_output(i).kus = kus;
    sim_output(i).V_crit = V_crit;
end

end

