clear all;
close all;
clc;
warning('off')


%% setup_copter_user:
Bicopter_type = 1;
Ihab_user = 1;
user = Ihab_user;
Copter_type = Bicopter_type;
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
PID_separated = 5;
fuzzy_logic = 6;
ADRC_SEP = 7;
ADRC_Kalman = 8;
PID_VAL = init_pid_val(Copter_type);
STSC_VAL = init_STSC_val(Copter_type);
[K,L,b,C] = init_ADRC_val(Copter_type);
[NLADRC_VAL,r,mult_obs,epsi] = init_Non_Linear_ADRC_val(Copter_type);
PID_VAL_SEP = init_pid_val_sep(Copter_type);
fuzzy_controller = init_fuzzy_controller(Copter_type);
% for adrc with nomral estimator:
[b_sep,wc_sep,wo_sep,eps_sep] = get_adrc_sep(Copter_type);
% for adrc with kalman
%[b_sep,wc_sep,wo_sep,eps_sep] = get_adrc_sep_kalman(Copter_type);

if Copter_type == Bicopter_type
    sat_max = [7;7;0.3;0.3];
    sat_min = [0;0;-0.3;-0.3];
    polarity = [-1;1;1;1];
else
    sat_max = [7;7;7;0.3];
    sat_min = [0;0;0;-0.3];
    polarity = [1;1;1;-1];
end
%% chose control method

%method = PID_controller;
method = STSC_controller;
%method = LADRC_controller;
%method = ADRC_SEP;
%method = PID_separated;
%method = fuzzy_logic;
%method = ADRC_Kalman;
tuning = false;
mannual_tuning = false;
method_angle_plane_sw = angle_sw; % = plane_sw
%method_angle_plane_sw = plane_sw;
euler_source = 1;
freq_cut_angvel_phi = 75;
freq_cut_angvel_theta = 50; % 100 hz 
freq_cut_angvel_psi = 75;
N_phi = 0.01;
S_phi = 0.001; % = 1/100*2*pi/f_max 
N_theta = 0.01;
S_theta = S_phi;
N_psi = 0.01;
S_psi = S_phi;
Noise_gain = 0.01;
Noise_gain = 1;
V_phi = 0.01; % the variance of the noise
V_theta = 0.01;
V_psi = 0.01;
C_air = 0.0025;


%
X_ref = 1;
Y_ref = 1;
% filtering the derivative :
if (Copter_type == 1)
    d_f_phi = 100;
    d_f_z = 100;
    d_f_theta = 10;
    d_f_psi = 25;
else
    d_f_phi = 25;
    d_f_z = 100;
    %d_f_theta = 25;
    d_f_theta = 100;
    d_f_psi = 25;
end
%%
if (Copter_type == 1)
    copter_mass = 0.482; % kg
    l = 0.135;
    h = 0.06;
    Ixx = 0.002290766613;
    Iyy = 0.000718997653;
    Izz = 0.002561189230;
else
    copter_mass = 414.287e-3; % tricopter mass here.
    l = 92.92e-3;
    I = [753.2,0,0;1.653,1253.393,0;139.549,-2.525,1635.023]*1e-6;
    saturation_power = 5;
    saturation_alpha_upper = 0.3;
    saturation_alpha_lower = -0.3;
    z_ref = 1;
    roll_ref = 0;
    yaw_ref = 0;
    pitch_ref = 0;
end
g = 9.81;

if Copter_type == Bicopter_type
    copter_path = '\bicopter_sim';
else
    if Copter_type ==Tricopter_type 
        copter_path = '\tricopter_sim';
    else
        copter_path = '\tricopter_modified_final';
    end
end
%%
[Para_min,Para_max] = parameters_min_max();
[State_step,State_3tau_max,State_3tau_min,State_over] = req_response();
open_system('Control_v2.slx');
model_wrk = get_param(bdroot, 'modelworkspace');
controllers = ["Control_v2/System/Controller/PID_Controller","Control_v2/System/Controller/super_twisting_controller1","Control_v2/System/Controller/ADRC_controller","Control_v2/System/Controller/NonLinearADRC","Control_v2/System/Controller/PID_seperated","Control_v2/System/Controller/fuzzy logic","Control_v2/System/Controller/ADRC_SEP","Control_v2/System/Controller/ADRC_Kalman"];
for i =1:length(controllers)
    set_param(controllers(i),'commented','on')
end
set_param(controllers(method),'commented','off')

if tuning
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
Y_max_phi = 0.35;
Y_min_phi = - 0.05;
Y_max_theta = 0.35;
Y_min_theta = - 0.05;
Y_max_psi = 1.1;
Y_min_psi = -0.2;