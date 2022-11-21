g = 9.81;             % m/s^2
m = 1573;             % kg
Iz = 3200;            % kg*m^2
a = 1.311;            % m
b = 1.539;            % m
caf_tire = 45000;     % N/rad/tire
car_tire = 70000;     % N/rad/tire
mu = 1.2;
dt = 0.01;            % s
caf = caf_tire*2;
car = car_tire*2;
tire_relaxation_length = 0.5; % m
min_V = 0;

L = a + b;            % m
Wf = m*g*b/L;         % N
Wr = m*g*a/L;         % N

kus = Wf/(caf) - Wr/(car);    % rad/g


car_params = struct;
car_params.g = g;
car_params.m = m;            
car_params.Iz = Iz;          
car_params.a = a;          
car_params.b = b;            
car_params.caf_tire = caf_tire;     
car_params.car_tire = car_tire;
car_params.caf = caf;
car_params.car = car;
car_params.L = L;
car_params.Wf = Wf;
car_params.Wr = Wr;
car_params.kus = kus;
car_params.mu = mu;
car_params.dt = dt;
car_params.tire_relaxation_length = tire_relaxation_length; % m
car_params.use_duggoff = 0;
car_params.use_tire_relaxation = 0;
car_params.min_V = min_V;