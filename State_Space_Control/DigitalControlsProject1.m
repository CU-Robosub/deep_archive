%% Clear old variables
clear all
close all
%clc


% % Define mapping from motors to inputs
% I1 = 0.377952;
% I2 = 0.28702;
% I3 = 0.03506;
% I4 = 0.162052;
% L = [0 0 1 1 0 0 0 0;
%     1 1 0 0 0 0 0 0;
%     0 0 0 0 1 1 1 1;
%     0 0 0 0 -I4 I4 -I4 I4;
%     0 0 I3 I3 I1 I1 -I1 -I1;
%     I1 -I1 -I2 I2 0 0 0 0];


%% Set up vehicle properties
% Center of Mass (Cg)
Cg = [0.5392,0.2286,0.2789]';
% Co Origin
Co = Cg;
rg = Co-Cg;


% Center of Bouyancy (Cb)
Cb = [0.5392,0.2286,0.3906]';


% Vehicle mass
m = 25.8618; %kg

% Vehicle Inertia from Cg
I = [0.7628 0.0008 0.0025;
    0.0008 1.1921 0.0068;
    0.0025 0.0068 1.1852];

Ix = 0.7628;
Iy = 1.1921;
Iz = 1.1852;
% % Mass Added (estimate from fluid friction force (SolidWorks Flow) 1m/s
% % velocity)
% Ma_x = 0.0702;
% Ma_y = 0.731;
% Ma_z = 0.463;
% % Defining rotaitonal mass to be the same "faces" as their corresponding
% % linear masses. I.E. yaw flow is over "y" plane (probably not the best
% % approximation, experimental data needed). 
% Ma_r = 0.731;
% Ma_p = 0.463;
% Ma_q = 0.731;

Ma_test = -diag([0.4*m, .75*m, .9*m, .9*Ix, .9*Iy, .8*Iz]);

% Define "Moment" matrix
% Rigid Body Matrix
Mb = [m*eye(3,3), zeros(3,3); 
    zeros(3,3),diag(diag(I))];
% Ma = -diag([Ma_x, Ma_y, Ma_z, Ma_r, Ma_p, Ma_q]);
% Mass Added Matrix
M = Mb+Ma_test;



% Define Gravity Matrix
g = 9.81; %m/s^2
% Difference between Cg and Cb
pq_bouyancy = Cg(3)-Cb(3);
% Vehicle weight in Newtons
W = g*m;
% Vehicle bouyancy in Newtons
B = -g*(32);
% Gravity Matrix
G = -diag([0,0,B+W,pq_bouyancy*W,pq_bouyancy*W,0]);

% Define Drag Matrix 

% Linear drags (estimated from fluide force (SolidWorks Flow) 1m/s
% velocity)
Force_x = -27.569;
Force_y = -67.911;
Force_z = -66.522;

% Defining rotaitonal drags to be the same "faces" as their corresponding
% linear drags. I.E. yaw flow is over "y" plane (probably not the best
% approximation, experimental data needed). 
Force_r = -67.911;
Force_p = -66.522;
Force_q = -67.911;

% Drag Matrix
D = -diag([Force_x, Force_y, Force_z, Force_r, Force_p, Force_q]);


%% Set up Continuous Model
% Model defined with all 12 dof

% Model dropped as it is uncontrollable, and unobservable

% Define State Space Model (Note C is an I since we want to see every state)
% A = [zeros(6,6), eye(6,6); -inv(M)*G, -inv(M)*D];
% B = [zeros(6,6), inv(M)]';
% C = [eye(12,12)];
% D = [zeros(12,6)];
% 
% model = ss(A,B,C,D);
% Co = ctrb(model);
% Ob = obsv(model);
% unco = length(A)-rank(Co);
% unob = length(A)-rank(Ob);
% stabilizable = rank([eig(A).*eye(12,12) B]);
% detectable = rank([eig(A).*eye(12,12); C]);

%% Set up Seperable SISO Models
% Model defined with 6x2dof siso models
unco = zeros(6,1);
unob = zeros(6,1);
A_SISO = cell(6,1);
B_SISO = cell(6,1);
C_SISO = cell(6,1);
D_SISO = cell(6,1);

Model_SISO = cell(6,1);

P = zeros(2,6);

for i = 1:6
    A_SISO{i} = [0, 1; -G(i,i)*1/(M(i,i)), -D(i,i)*1/(M(i,i))];
    B_SISO{i} = [0; 1/(M(i,i))];
    % Desired output is position
    C_SISO{i} = [1 0];
    D_SISO{i} = zeros(1);
    Model_SISO{i} = ss(A_SISO{i},B_SISO{i},C_SISO{i},D_SISO{i});
    % Check models for controllability and observability
    unco(i) = length(A_SISO{i})-rank(ctrb(Model_SISO{i}));
    unob(i) = length(A_SISO{i})-rank(obsv(Model_SISO{i}));
    % Check poles of each model
    P(:,i) = pole(Model_SISO{i});
end


%% Define Descrete Models based on T
% Define Time Delay
% T = 1/8; % DVL Update in Seconds
% T = 1/100
 T = 1/40; % State Estimator Update in Seconds

D_Model_SISO = cell(6,1);
unco_D = zeros(6,1);
unob_D = zeros(6,1);

P_D = zeros(2,6);

for i = 1:6
    D_Model_SISO{i} = c2d(Model_SISO{i},T);
    % Check discrete models for controllability and observability
    CoD = ctrb(D_Model_SISO{i});
    ObD = obsv(D_Model_SISO{i});
    unco_D(i) = length(A_SISO{i})-rank(CoD);
    unob_D(i) = length(A_SISO{i})-rank(ObD);
    % Check poles of each model
    P_D(:,i) = pole(D_Model_SISO{i});
    % If the model is uncontrollable or unobservable check if it is
    % stabilizeable or detectable
    stabilizable(i) = rank([eig(D_Model_SISO{i}.A).*eye(2,2) D_Model_SISO{i}.B]);
    detectable(i) = rank([eig(D_Model_SISO{i}.A).*eye(2,2); D_Model_SISO{i}.C]);
end

%% Define Control Gain K
% Set desired poles given omega_n = 0.45, zeta = 0.6
if(T == 1/40)
    p_test = [0.993-0.00894i, 0.993+0.00894i];
elseif(T == 1/8)
    p_test = [0.96-0.048i, 0.96+0.048i];
end
% K_10 = cell(3,1);
% K_20 = cell(3,1);
% K_40 = cell(3,1);
K_test = cell(6,1);
N = cell(6,1);

for i = 1:6
%    K_10{i} = place(D_Model_SISO{i}.A, D_Model_SISO{i}.B, p_des_overshoot10);
%    K_20{i} = place(D_Model_SISO{i}.A, D_Model_SISO{i}.B, p_des_overshoot20);
%    K_40{i} = place(D_Model_SISO{i}.A, D_Model_SISO{i}.B, p_des_overshoot40);
   K_test{i} = place(D_Model_SISO{i}.A, D_Model_SISO{i}.B, p_test);
   N_vec = inv([D_Model_SISO{i}.A-eye(size(D_Model_SISO{i}.A)), D_Model_SISO{i}.B; D_Model_SISO{i}.C, [0]])*[0;0;1];
   N{i} = N_vec(3)+K_test{i}*N_vec(1:2);
end

% K_test{6} = place(D_Model_SISO{6}.A, D_Model_SISO{6}.B, p_test);
% N_vec = inv([D_Model_SISO{6}.A-eye(size(D_Model_SISO{6}.A)), D_Model_SISO{6}.B; D_Model_SISO{6}.C, [0]])*[0;0;1];
% N{6} = N_vec(3)+K_test{6}*N_vec(1:2);

%% Simulate step response to gain K
IC = [0;0];
sim_time = 20;
sim('X_Model.slx')
sim('Y_Model.slx')
sim('Z_Model.slx')
sim('Yaw_Model.slx')

step_information{1} = stepinfo(y_1,t_1,1);
step_information{2} = stepinfo(y_2,t_2,1);
step_information{3} = stepinfo(y_3,t_3,1);
step_information{6} = stepinfo(y_6,t_6,pi/2);


%% Plot Step Response with Gain K
figure
hold on
plot(t_1, r_1,'Linewidth', 1.5)
stairs(t_1,y_1,'Linewidth', 1.5)
legend('Reference','North','Location', 'southeast')
xlabel('Time (s)')
ylabel('Response (m)')
title(['Step response for a North position change of one meter', newline,'with feedback control'])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)
hold off

figure
hold on
plot(t_2, r_2,'Linewidth', 1.5)
stairs(t_2,y_2,'Linewidth', 1.5)
legend('Reference', 'East','Location', 'southeast')
xlabel('Time (s)')
ylabel('Response (m)')
title(['Step Response for an East position change of one meter', newline, 'with feedback control'])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)
hold off

figure
hold on
plot(t_2, r_2,'Linewidth', 1.5)
stairs(t_3,y_3,'Linewidth', 1.5)
legend('Reference','Down','Location', 'southeast')
xlabel('Time (s)')
ylabel('Response (m)')
title(['Step Response for an Down position change of one meter', newline, 'with feedback control'])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)
hold off

figure
hold on
plot(t_6, r_6,'Linewidth', 1.5)
stairs(t_6,y_6,'Linewidth', 1.5)
legend('Reference','Yaw','Location', 'southeast')
xlabel('Time (s)')
ylabel('Response (rad)')
title(['Step Response for a Yaw position change of pi/2 radian', newline, 'with feedback control'])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)
hold off


%% Define Observer Gain L

% 2x Poles
% if(T == 1/40)
%     p_test_obs = [0.986-0.0178i, 0.986+0.0178i];
% elseif(T == 1/8)
%     p_test_obs = [0.93-0.084i, 0.93+0.084i];
% end

% % 10x ples (damping ratio 1%, tr_des = .4)
% if(T == 1/40)
%     p_test_obs = [0.912-0.0616i, 0.912+0.0616i];
% elseif(T == 1/8)
%     p_test_obs = [0.601-0.211i, 0.601+0.211i];
% end



% 100x poles (damping ratio 1%, tr_des = .04)
if(T == 1/40)
    p_test_obs = [0.3174+0.245i,0.3174-0.245i];
elseif(T == 1/8)
    p_test_obs = [-0.0108+0.00257i,-0.0108-0.00257i];
end

M = cell(6,1);
L_test = cell(6,1);
for i = 1:6
    L_test{i} = place((D_Model_SISO{i}.A)', (D_Model_SISO{i}.C)', p_test_obs)';
       
    M{i} = D_Model_SISO{i}.B*N{i};
end


%% Simulate with Observer Gain L
IC = [0.02; 0.03];
sim_time = 5;
sim('X_Model.slx')
sim('X_Model_OBS.slx')
sim('Y_Model_OBS.slx')
sim('Z_Model_OBS.slx')
sim('Yaw_Model_OBS.slx')

step_information_obs{1} = stepinfo(y_1_obs,t_1_obs,1);
step_information_obs{2} = stepinfo(y_2_obs,t_2_obs,1);
step_information_obs{3} = stepinfo(y_3_obs,t_3_obs,1);
step_information_obs{6} = stepinfo(y_6_obs,t_6_obs,pi/2);

%% Plot Step Response North with full state feedback
figure
hold on
plot(t_1, r_1,'Linewidth', 1.5)
stairs(t_1,y_1,'Linewidth', 1.5)
legend('Reference','North','Location', 'southeast')
xlabel('Time (s)')
ylabel('Response (m)')
title(['Step response for a North position change of one meter', newline, 'with feedback control, and offset initial conditions'])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)
hold off

figure
hold on
plot(t_1_obs, r_1_obs,'Linewidth', 1.5)
stairs(t_1_obs,y_1_obs,'Linewidth', 1.5)
legend('Reference','North','Location', 'southeast')
xlabel('Time (s)')
ylabel('Response (m)')
title(['Step response for a North position change of one meter', newline, 'with full state feedback control, and offset initial conditions'])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)
hold off

figure
hold on
stairs(t_1_obs(1:40),y_1_obs(1:40)-y_hat_1_obs(1:40), 'Linewidth', 1.5)
legend('y-y_{hat}')
xlabel('Time (s)')
ylabel('Esitmator Error (m)')
title(['Estimator Error given non-zero intial conditions', newline, 'for North position change'])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)

%% Plot Step Response East with full state feedback
figure
hold on
plot(t_2_obs, r_2_obs,'Linewidth', 1.5)
stairs(t_2_obs,y_2_obs,'Linewidth', 1.5)
legend('Reference','East','Location', 'southeast')
xlabel('Time (s)')
ylabel('Response (m)')
title(['Step response for a East position change of one meter', newline, 'with full state feedback control, and offset initial conditions'])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)
hold off

figure
hold on
stairs(t_2_obs(1:40),y_2_obs(1:40)-y_hat_2_obs(1:40), 'Linewidth', 1.5)
legend('y-y_{hat}')
xlabel('Time (s)')
ylabel('Esitmator Error (m)')
title(['Estimator Error given non-zero intial conditions', newline 'for East position change'])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)

%% Plot Step Response Down with full state feedback
figure
hold on
plot(t_3_obs, r_3_obs,'Linewidth', 1.5)
stairs(t_3_obs,y_3_obs,'Linewidth', 1.5)
legend('Reference','Down','Location', 'southeast')
xlabel('Time (s)')
ylabel('Response (m)')
title(['Step response for a Down position change of one meter', newline, 'with full state feedback control, and offset initial conditions'])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)
hold off

figure
hold on
stairs(t_3_obs(1:40),y_3_obs(1:40)-y_hat_3_obs(1:40), 'Linewidth', 1.5)
legend('y-y_{hat}')
xlabel('Time (s)')
ylabel('Esitmator Error (m)')
title(['Estimator Error given non-zero intial conditions', newline,'for Down position change'])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)

%% Plot Step Response Yaw with full state feedback
figure
hold on
plot(t_6_obs, r_6_obs,'Linewidth', 1.5)
stairs(t_6_obs,y_6_obs,'Linewidth', 1.5)
legend('Reference','Yaw','Location', 'southeast')
xlabel('Time (s)')
ylabel('Response (m)')
title(['Step response for a Yaw position change of one meter', newline,'with full state feedback control, and offset initial conditions'])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)
hold off

%%
figure
hold on
stairs(t_6_obs(1:40),y_6_obs(1:40)-y_hat_6_obs(1:40), 'Linewidth', 1.5)
legend('y-y_{hat}')
xlabel('Time (s)')
ylabel('Esitmator Error (m)')
title(['Estimator Error given non-zero intial conditions', newline,'for Yaw position change'])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)

%%
figure
hold on
stairs(t_6_obs(1:80),u_6_obs(1:80), 'Linewidth', 1.5)
legend('u')
xlabel('Time (s)')
ylabel('Motor Output (Nm)')
title(['Motor Output given non-zero intial conditions', newline,'for Yaw position change'])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)


%%
z = tf('z',T)
for ii = 1:6
    [num den] = ss2tf(D_Model_SISO{ii}.A,D_Model_SISO{ii}.B,D_Model_SISO{ii}.C,D_Model_SISO{ii}.D);
    H_D{ii} = tf(num,den,T)
    
    D_D{ii} = -K_test{ii}*inv(z*eye(2,2)-D_Model_SISO{ii}.A+D_Model_SISO{ii}.B*K_test{ii}+L_test{ii}*D_Model_SISO{ii}.C)*L_test{ii}
end
%%
figure
hold on
pzmap(D_D{6})
title('Pole-Zero Map, D_{Yaw}')
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)
hold off
figure
hold on
pzmap(D_D{1})
title('Pole-Zero Map, D_{Down}')
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 12)
