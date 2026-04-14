% DCTA PROJECT 
close all;
clear;
clc;

%% Data 

Ra = 10; 
La = 0.18;
Kt = 7.68e-3;
Km = 7.68e-3;
Kg = 70;
eta_m = 0.85;
eta_g = 0.90;
Jeq = 0.0022;
Beq = 0.015;
K1 = 1.5;
J1 = 0.025;
B1 = 0.05;


A = [0 1 0 0; -K1/Jeq -Beq/Jeq-(eta_m*eta_g*Kg^2*Kt*Km)/(Ra*Jeq) K1/Jeq 0; ... 
    0 0 0 1; K1/J1 0 -K1/J1 -B1/J1];

B = [0; (eta_m*eta_g*Kg*Kt)/(Ra*Jeq); 0; 0];

C = [0, 0, 1, 0;   % theta_1   
     0, 0, 0, 1];  % theta_dot_1 

D = [0, 0]';

%% Create system transfer function

sys = ss(A, B, C, D);
G = tf(sys);
G_vel = tf(ss(A, B, C(2,:), D(2)));  % Va → theta_dot_1
G_pos = tf(ss(A, B, C(1,:), D(1)));  % Va → theta_1

zeros_vel = tzero(G_vel)
poles_vel = pole(G_vel)

zeros_pos = tzero(G_pos)
poles_pos = pole(G_pos)

figure
step(G_pos);
grid on
figure
step(G_vel);
grid on

figure
bode(G_pos)
grid on
figure
bode(G_vel)
grid on

%% Inner loop
% controlSystemDesigner(G_vel)
gain_vel=20;
pole1_vel=0.02;
zero1_vel=0.15;
zero2_vel=0.03;
tau_f=0.02; 
% PID formulation
ki_vel=gain_vel;
kp_vel=(zero1_vel+zero2_vel)*ki_vel-ki_vel*pole1_vel;
kd_vel=(zero1_vel*zero2_vel)*ki_vel-kp_vel*pole1_vel;
s = tf('s');
R_vel = gain_vel * (1 + zero1_vel*s) * (1 + zero2_vel*s) / (s * (1 + pole1_vel*s));

% Check margins on Bode
L_inner=R_vel*G_vel;
margin(L_inner)
% Confirm with closed-loop poles
sys_cl_inner = feedback(L_inner, 1);
disp(pole(sys_cl_inner))

% Complementary sensitivity funct
F_inner = feedback(L_inner,1);
figure
step(F_inner)
grid on
stepinfo(F_inner)
figure
bode(F_inner)
grid on
pole(F_inner) 

% Sensitivity funct
S_inner = feedback(1,L_inner);
bode(S_inner)
grid on

wcinner=6.3;

%% Outer loop

wcouter=1.3;
mu_q=dcgain(F_inner);
Kpo=wcouter/mu_q;

R_pos=Kpo*s/s;

% L_outer
s = tf('s');
L_outer = (Kpo/s)*F_inner;
figure
margin(L_outer)
grid on

% Confirm with closed-loop poles
sys_cl_outer = feedback(L_outer, 1);
disp(pole(sys_cl_outer))

% Complementary sensitivity funct
F_outer = feedback(L_outer,1);
figure
step(F_outer)
grid on
stepinfo(F_outer)
figure
bode(F_outer)
grid on
pole(F_outer) 

% Sensitivity funct
S_outer = feedback(1,L_outer);
bode(S_outer)
grid on

%% PART 2
Ts=0.02;
TdelayADCVel=9e-6;
TdelayADCPos=11e-6;

% Discretize controller with Tustin
Rzvel = c2d(R_vel, Ts, 'tustin');
Gzvel = c2d(G_vel, Ts, 'zoh');

Rzpos = c2d(R_pos, Ts, 'tustin');
Gzpos = c2d(G_vel/s, Ts, 'zoh');

% Closed-loop discrete
Lzinner = series(Rzvel, Gzvel);
Fzinner = feedback(Lzinner,1);

Lzouter = series(Rzpos, Gzpos);
Fzouter = feedback(Lzouter,1);

% Evaluate performance
figure; margin(Lzinner); grid on; 
figure; step(Fzinner, 0:Ts:2); grid on; title('EX1: Discrete Closed-loop Step Response');

figure; margin(Lzouter); grid on; 
figure; step(Fzouter, 0:Ts:2); grid on; title('EX1: Discrete Closed-loop Step Response');

% Phase loss due to the sampling
phi_lostinner= 0.5*Ts*wcinner;
phi_lostinner = phi_lostinner*180/pi;


phi_lostouter= 0.5*Ts*wcouter;
phi_lostouter = phi_lostouter*180/pi;

%% Anti Aliasing Filter
ws=(2*pi)/Ts;     %rad/s
wn=0.5*ws;  %rad/s

w_AA=wn/3;  %rad/s
tau_AA=1/w_AA; %s
s = tf('s');
G_AA = 1 / ((tau_AA*s+1)^2);

Linner_AA=L_inner*G_AA;
Louter_AA=L_outer*G_AA;
figure;
margin(Linner_AA)
figure;
margin(Louter_AA)

%%
% Choose practical ADC for inner loop: 8 or 12 bits
K_ampl=2.75; %V/V
dVa_dw=K_ampl*10*1e-3; %V/(deg/s)
Vfs=3.3;
% Resolution with 8-bit
Res_8=Vfs/(2^8-1)*1/(dVa_dw)  %(deg/s)/bit
% Resolution with 12-bit
Res_12=Vfs/(2^12-1)*1/(dVa_dw) %(deg/s)/bit
Vref = 3.3;
N_choose9 = 9; 
LSB_9 = Vref/(2^N_choose9-1);
fprintf('Using 9-bit ADC -> LSB = %.6f V\n', LSB_9);
N_choose11 = 11; 
LSB_11 = Vref/(2^N_choose11-1);
fprintf('Using 11-bit ADC -> LSB = %.6f V\n', LSB_11);
K_transducerVel=(10e-3)/(pi/180); %[V/(rad/s)]
K_opampl=2.75; %V/V
bias_opampl=1.65; %V

% DAC
LSB_quantizer = 12/(2^N_choose9 - 1);
% Power amplifier
K_powerampli = 0.023; % [V/bit]
bias_powerampli = -12; % [V]
