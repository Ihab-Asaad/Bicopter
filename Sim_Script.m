clear all;
close all;
clc;
warning('off')

%% setup_path
bicopter_path = strcat(pwd,'\bicopter_sim\Bicopter_DataFile14.m');
file_name = strcat(pwd,'\Control_v2');
%%
time_delay = 1e-6;
step_size = '0.001';
Stop_time = '10.0';
Step_size = step_size;
Copter = init_Copter_sim(Stop_time,step_size,'normal','1e-3','off','off','Fixed-step','FixedStepDiscrete');
%% controllers
angle_sw = 1;
plane_sw = 2;
PID_controller = 1;
STSC_controller = 2;
LADRC_controller = 3;
NLADRC_controller = 4;
fuzzy_logic = 5;
ADRC_SEP = 6;
ADRC_Kalman = 7;
% initialization:
PID_VAL_SEP = init_pid_val_sep();
STSC_VAL = init_STSC_val();
[K,L,b,C] = init_ADRC_val();
[NLADRC_VAL,r,mult_obs,epsi] = init_Non_Linear_ADRC_val();
fuzzy_controller = init_fuzzy_controller();
% for adrc with nomral estimator:
[b_sep,wc_sep,wo_sep,eps_sep] = get_adrc_sep();
% for adrc with kalman
%[b_sep,wc_sep,wo_sep,eps_sep] = get_adrc_sep_kalman();

sat_max = [7;7;0.3;0.3];
sat_min = [0;0;-0.3;-0.3];
polarity = [-1;1;1;1];

%% choose control method

method = PID_controller;
%method = STSC_controller;
%method = LADRC_controller;
%method = ADRC_SEP;
%method = fuzzy_logic;
%method = ADRC_Kalman;

sim_editor = false; % if true, show bicopter body during simulation

% euler angle control or XY control:
method_angle_plane_sw = angle_sw; % = plane_sw
%method_angle_plane_sw = plane_sw;

% choose euler source: 1 : euler from "quaternions to rotation block",
% 2: euler calculated from angular velocity
euler_source = 1;

% Constants:
freq_cut_angvel_phi = 75;
freq_cut_angvel_theta = 50; % 100 hz 
freq_cut_angvel_psi = 75;
N_phi = 0.01;
S_phi = 0.001; % = 1/100*2*pi/f_max 
N_theta = 0.01;
S_theta = S_phi;
N_psi = 0.01;
S_psi = S_phi;
Noise_gain = 1; % 0.01
V_phi = 0.01; % the variance of the noise
V_theta = 0.01;
V_psi = 0.01;
C_air = 0.0025;

% for scope:
Y_max_phi = 0.35;
Y_min_phi = - 0.05;
Y_max_theta = 0.35;
Y_min_theta = - 0.05;
Y_max_psi = 1.1;
Y_min_psi = -0.2;

% references:
X_ref = 1;
Y_ref = 1;
Z_ref = 1;


% filtering the derivative :
d_f_phi = 100;
d_f_z = 100;
d_f_theta = 10;
d_f_psi = 25;
%%
copter_mass = 0.482; % kg
l = 0.135;
h = 0.06;
Ixx = 0.002290766613;
Iyy = 0.000718997653;
Izz = 0.002561189230;
g = 9.81;


copter_path = '\bicopter_sim';
%% limits for tuning:
[Para_min,Para_max] = parameters_min_max();
[State_step,State_3tau_max,State_3tau_min,State_over] = req_response();
%% 
open_system('Control_v2.slx');
model_wrk = get_param(bdroot, 'modelworkspace');
controllers = ["Control_v2/System/Controller/PID_Controller","Control_v2/System/Controller/super_twisting_controller1","Control_v2/System/Controller/ADRC_controller","Control_v2/System/Controller/NonLinearADRC","Control_v2/System/Controller/fuzzy logic","Control_v2/System/Controller/ADRC_SEP","Control_v2/System/Controller/ADRC_Kalman"];
for i =1:length(controllers)
    set_param(controllers(i),'commented','on')
end
set_param(controllers(method),'commented','off')

if sim_editor
    set_param('Control_v2','SimMechanicsOpenEditorOnUpdate','off')
end
mux_bicopter = get_param('Control_v2/System/Bicopter_Mux','PortHandles');
% mux_tricopter = get_param('Control_v2/System/Tricopter_Mux','PortHandles');
% mux_m_tricopter = get_param('Control_v2/System/Tricopter_Mux_m','PortHandles');

angle_plane_sw = get_param('Control_v2/System/angle_plane_sw','PortHandles');
mux_angle_ref_angle = get_param('Control_v2/System/angle_ref_angle','PortHandles');
mux_plane_ref_angle = get_param('Control_v2/System/plane_ref_angle','PortHandles');

% sw = get_param('Control_v2/System/Copter_switch','PortHandles');
    
cd 'bicopter_sim'  % change path before running the simulation:
sw_copter = 1;
model_wrk.FileName = bicopter_path;
set_param('Control_v2/System/Bicopter','commented','off');
set_param('Control_v2/System/Bicopter_Mux','commented','off');
% set_param('Control_v2/System/Tricopter','commented','on');
% set_param('Control_v2/System/Tricopter_Mux','commented','on');
% set_param('Control_v2/System/Tricopter_modified','commented','on');
% set_param('Control_v2/System/Tricopter_Mux_m','commented','on');
    %set_param('Control_v2/Bicopter_Mux','Inputs','2');
    %set_param('Control_v2/Tricopter_Mux','Inputs','6');
    %set_param('Control_v2/switch_copter', 'sw', '1');
try
  add_line('Control_v2',mux_bicopter.Outport(1),sw.Inport(2));
catch exception
  fprintf('already done\n');
end
try
  delete_line('Control_v2',mux_tricopter.Outport(1),sw.Inport(3));
catch exception
  fprintf('already done\n');
end

try
  delete_line('Control_v2',mux_m_tricopter.Outport(1),sw.Inport(4));
catch exception
  fprintf('already done\n');
end



if method_angle_plane_sw == angle_sw
    try
        delete_line('Control_v2/System',mux_plane_ref_angle.Outport(1),angle_plane_sw.Inport(3));
    catch
        fprintf('already done\n');
    end
    try
        add_line('Control_v2/System',mux_angle_ref_angle.Outport(1),angle_plane_sw.Inport(2));
    catch
        fprintf('already done\n');
    end
    
else
    try
        delete_line('Control_v2/System',mux_angle_ref_angle.Outport(1),angle_plane_sw.Inport(2));
    catch
        fprintf('already done\n');
    end
    try
        add_line('Control_v2/System',mux_plane_ref_angle.Outport(1),angle_plane_sw.Inport(3));
    catch
        fprintf('already done\n');
    end
end
reload(model_wrk);% to save the new file name to model workspace
%%
simOut = sim(file_name,Copter);
%%
