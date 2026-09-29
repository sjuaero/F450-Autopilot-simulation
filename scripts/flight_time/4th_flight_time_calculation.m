%% parameters

R = 0.119; %[m]
fm = 0.4;
rho = 1.225; %[kg/m^3]
U_batt = 5000; %[mAh]
V_batt = 14.8; %[V]
m = 1.5; %[kg]
g = 9.8; %[m/s^2]


%One motor Thrust in hover
T_prop = 1/4*m*g


%P_ideal
P_ideal = (T_prop^1.5)/sqrt(2*rho*R^2*pi)

%P_real
P_real = 4*P_ideal/fm

%E_batt
E_batt = U_batt*V_batt/1000

%Maximum flight time
time = E_batt/P_real*60 %[MIN]

fprintf('Drone can hover %0.3f minutes\n', time)