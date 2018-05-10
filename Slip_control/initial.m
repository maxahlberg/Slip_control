%SLIP CONTROL OF ROAD OR RAIL VEHICLE
 clc
 clear all
 close all
 iter = 1;
 PP=50;
 ii =1;
 a_P_T_values = [];
 a_times = [];
 a_brake_dist = [];
for kk = 1:iter

global Veh
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Choose input parameters
Vehicle = 'Road'            % set Road or Rail for the vehicle parameters
mu_select =1;              % set friction to mu_select = 1 (dry road), 2 (wet
% road) or 3 (snow) for road and 1 for rail
Task = 2;
dt = 0.001;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle parameters
switch Vehicle
    case {'Road'}
        % Initial values for road vehicle
        Veh.mc  = 395; % sprung mass of a quarter car in kg
        Veh.mb = 5;
        Veh.mw = 25;%40; % unsprung mass of the quarter car in kg
        Veh.Jw = 1.7; % inertia of wheel in kgm^2
        Veh.kc = 300000; %bushing
        Veh.kb = 22000;%21000; % main spring coefficient in N/m
        Veh.kw = 2.4e5;%150000; % tire spring coefficient in N/m
        Veh.cc = 20;
        Veh.cb = 4500;%1500; % main damping coefficient in Ns/m
        Veh.cw = 0;%100; % tire damping coefficient in Ns/m
        Veh.Cs = 70000; % longitudinal tire stiffness in N/rad
        Veh.Cx = [1.6 1.65 1.1]; % Magic Formula tire shape factor
        Veh.Bx = [9 10 18]; % Magic Formula stiffness factor
        Veh.Dx= 9.81*(Veh.mc+Veh.mb+Veh.mw); % Magic Formula parameter
        Veh.Ex=0; % Magic Formula parameter
        Veh.k_V_low0=770*1; % low speed damping parameter in Ns/m
        Veh.V_low=2.5; % speed threshold in m/s
        Veh.sigma_min=0.02; % min relaxation length in m
        Veh.sigma_k0=0.1; % relaxation length in m
        Veh.Eps_F=0.01;  % Magic Formula division parameter
        Veh.fr = 0.008; % rolling resistance coefficient in -
        Veh.A = 2.2; % frontal cross-section area in m
        Veh.cw = 0.3; % aerodynamic drag coefficient in --
        Veh.r = 0.3; % rolling radius for tire in m
        Veh.mu = [1 0.7 0.3]; % friction coefficients [snow, rain and dry]
        tire_leg={'\mu = 1','\mu = 0.7','\mu = 0.3'};
        switch mu_select
            case {1}
                Veh.Bx = Veh.Bx(1);
                Veh.Cx = Veh.Cx(1);
                Veh.mu = Veh.mu(1);
            case {2}
                Veh.Bx = Veh.Bx(2);
                Veh.Cx = Veh.Cx(2);
                Veh.mu = Veh.mu(2);
            case {3}
                Veh.Bx = Veh.Bx(3);
                Veh.Cx = Veh.Cx(3);
                Veh.mu = Veh.mu(3);
        end
        Veh.C_fk=Veh.Bx*Veh.Cx*Veh.Dx;
        
    case {'Rail'}
        % Initial values for rail vehicle
        Veh.mc  = 4375; % car body of a eighth car body in kg
        Veh.mb = 1500;  % quarter mass of the bogie in kg
        Veh.mw = 3000; % unsprung mass of half a wheel set in kg
        Veh.Jw = 150; % inertia of wheel in kgm^2
        Veh.kc = 5*10^6; % secondary spring coefficient in N/m
        Veh.kb = 20*10^6; % primary spring coefficient in N/m
        Veh.kw = 2400e6; % wheel spring coefficient in N/m
        Veh.cc = 40*10^3; % secondary damping coefficient in Ns/m
        Veh.cb = 70*10^3; % primary damping coefficient in Ns/m
        Veh.cw = 0; % wheel damping coefficient in Ns/m
        Veh.Cs = 2400e6; % longitudinal tire stiffness in N/rad
        Veh.Cx = 1.0; % Magic Formula tire shape factor
        Veh.Bx = 50; % Magic Formula stiffness factor
        Veh.Dx= 9.81*(Veh.mc+Veh.mb+Veh.mw); % Magic Formula parameter
        Veh.Ex=0; % Magic Formula parameter
        Veh.k_V_low0=770*1; % low speed damping parameter in Ns/m
        Veh.V_low=2.5; % speed threshold in m/s
        Veh.sigma_min=0.02; % min relaxation length in m
        Veh.sigma_k0=0.2; % relaxation length in m
        Veh.Eps_F=0.01;  % Magic Formula division parameter
        Veh.C_fk=Veh.Bx*Veh.Cx*Veh.Dx;
        Veh.fr = 0.002; % rolling resistance coefficient in -
        Veh.A = 10; % frontal cross-section area in m
        Veh.cw = 0.26; % aerodynamic drag coefficient in --
        Veh.r = 0.46; % rolling radius for tire in m
        Veh.mu = 0.2; % friction coefficients [snow, rain and dry]
        tire_leg={'\mu = 0.2'};
        mu_select = 1;
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Driver parameters
Dri.v_lim = 90; %in km/h
Dri.t_const = 3; %in seconds
Dri.Ka = 1;  % throttle gain (0-100%) of driver
Dri.Kb = -1; % braking gain (0-100%) of driver

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control parameters
T_max = (Veh.mc+Veh.mb+Veh.mw)*9.81*Veh.r;
% K_em = 0.4*T_max;
% K_brake = 0.5*T_max;
% Kp_em = 0.4*T_max;
% Kp_brake = 0.5*T_max;
% Ki_em = 1;
% Ki_brake = 1;
% Kd_em = 1;
% Kd_brake = 1;

%% tire model - force-slip-curves
slip0 = -1:0.01:2;
mu_plot = diag(Veh.mu)*sin(atan(slip0'*Veh.Bx)*diag(Veh.Cx))';
% Fz_max = [mu';-mu'].*(m+mt)*9.81;grid on,
hold on
plot(slip0,mu_plot,'LineWidth',2),grid on, hold on
axis([-1 2 -1.1 1.1])
%plot([a(1) a(end)],[1;1]*Fz_max(1),[a(1) a(end)],[1;1]*Fz_max(2),[a(1) a(end)],[1;1]*Fz_max(3),[a(1) a(end)],[1;1]*Fz_max(4),[a(1) a(end)],[1;1]*Fz_max(5),[a(1) a(end)],[1;1]*Fz_max(6),'LineStyle','--','LineWidth',2,'Color',[0 0 0])
xlabel('longitudinal slip \kappa/rad')
ylabel('longitudinal force f_x/N')
legend(tire_leg(mu_select),'Location','NorthWest')

%%
% definition of state-space model for three mass quarter car model
% z(1)=! xc
% z(2)=! xcd = zd(1)
% z(3)=! xb
% z(4)=! xbd = zd(3)
% z(5)=! xw
% z(6)=! xwd = zd(5)
% this gives:
% zd(1)= z(2)   = xcd
% zd(2)= zdd(1) = xcdd
% zd(3)= z(4)   = xbd
% zd(4)= zdd(3) = xbdd
% zd(5)= z(6)   = xwd
% zd(6)= zdd(5) = xwdd

AA = [0 1 0 0 0 0; -Veh.kc/Veh.mc -Veh.cc/Veh.mc Veh.kc/Veh.mc Veh.cc/Veh.mc 0 0; 0 0 0 1 0 0; Veh.kc/Veh.mb Veh.cc/Veh.mb -(Veh.kc+Veh.kb)/Veh.mb -(Veh.cc+Veh.cb)/Veh.mb Veh.kb/Veh.mb Veh.cb/Veh.mb; 0 0 0 0 1 0; 0 0 Veh.kb/Veh.mw Veh.cb/Veh.mw -(Veh.kb+Veh.kw)/Veh.mw -(Veh.cb+Veh.cw)/Veh.mw];
BB = [0 0; 0 0; 0 0; 0 0; 0 0; Veh.kw/Veh.mw Veh.cw/Veh.mw];
% CC = eye(6); % use for motions
% DD = [0 0;0 0;0 0;0 0;0 0;0 0;]; % use for motions
CC = [0 0 0 0 -Veh.kw -Veh.cw]; % use for dynamic load
DD = [Veh.kw Veh.cw]; % use for dynamic load

% sim('SlipModel')
%
% visualise();



%% EGEN KOD 
%close all; clc

slip_opt=0.14;
P=1000; %1340, 10, 50 on wet 3.7171 s
I=10; %1340,580,50 on wet 3.7004 s
D=70; %1340,580,60 wet 3.6978 s
N=100; %1340,580,60,240 wet 3.6877 s

Pabs=130; %700,10,50 best road wet 48.9407 m
Iabs=30; %700,351,50 best wet 48.9228 m
Dabs=10; %700,351,50 wet 48.9260 m :/
Nabs=240; %700,351,50,100 wet 48.9242
ARW = 0.5;

% Find time for acceleration and braking
%close all; clc

sim('slip_model_Student')


%% 
%close all; clc

% figure
% plot(omega); hold on; plot(vx);
% title('Velocity Output with PD-Controller')
% xlabel('Time [s]')
% ylabel('Velocity [m/s]')
% legend('Wheel speed','Longitudinal vehicle speed','Location','NorthEast')
% grid on
% axis([0 22 0 30])

%saveas(gcf,'3_c_PD','png')


time=vx.Time;
Vx=vx.Data;
position=pos.Data;


for i=1:length(Vx)
    if Vx(i) >= 25
        a=i;
        break
    end
end

for i=10:length(Vx)
    if Vx(i) <= 0
        b=i;
        break
    end
end

t_b=time(a)+3;          % Time when braking starts

for i=1:length(time)
    if time(i) >= t_b
        c=i;
        break
    end
end

braking_distance=position(b)-position(c);

switch mu_select
    case {1}
        disp('The road is dry')
    case {2}
        disp('The road is wet')
    case {3}
        disp('The road is snowy')
end


disp(['Accelerations time is ' num2str(time(a)) ' seconds.'])
disp(['Braking distance is ' num2str(braking_distance) ' meters.'])


%T_acc_gain=T_max*1.05
%T_brake_gain=T_max*1.03


% Plotting the slip

% figure(2); 
% plot(omega);
% hold on
% plot(vx)
% grid on; 
% title('Velocity'); 
% xlabel('Time [s]'); 
% ylabel('Velocity [m/s]');
% axis([0 time(b-10) 0 30])
% legend('Wheel speed','Vehicle velocity')

% 
% figure(3); plot(slip); grid on; title('Slip of Wheel'); xlabel('Time [s]'); ylabel('Slip');
% axis([0 time(b-10) -0.05 0.6])
% hold on
% %end
% plot([0 time(b-10)],[0.14 0.14])
% legend('Wheel Slip','Optimal Slip')
% legend('Dry','Wet', 'Snowy','Position','NorthEast')
% saveas(gcf,'PID_rail_slip','png')


%saveas(gcf,'PID_rail_vel','png')
% figure
% plot(ABS_T)
% legend('ABS Torque')
% axis([8.6 12.2 -1500 3000])

a_times(kk) = time(a);
a_brake_dist(kk) = braking_distance;
a_P_T_values(kk) = PP;
PP = PP + 10;
end

best_acc = min(a_times);
best_stop = min(a_brake_dist);
for jj = 1:iter
    if a_times(jj) == best_acc
        PP_time_spot = jj;
    end
    if a_brake_dist(jj) == best_stop
        PP_stop_spot = jj;
    end
end
PP_val_acc = a_P_T_values(PP_time_spot);
PP_val_brake = a_P_T_values(PP_stop_spot);

disp(['The best acc. time is: ', num2str(best_acc),', with a N_tcs at: ', num2str(PP_val_acc)])

disp(['The best brake dist is: ', num2str(best_stop),', with a N_abs at: ', num2str(PP_val_brake)])


%%
%Task 4.c
% Ax = ax.Data;
% mu_utilized = Ax/9.81;
% 
% figure(2)
% plot(time(1:b), mu_utilized(1:b))
% axis([0 30 -0.6 0.6])
% hold on
% %end
% legend('Rail','Position','NorthEast')
% %legend('Dry','Wet', 'Snowy','Position','NorthEast')
% grid on
% title('Utilized Friction Plot')
% xlabel('Time [s]')
% ylabel('Utilized friction [ ]')




