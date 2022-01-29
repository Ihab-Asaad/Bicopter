function [State_step,State_3tau_max,State_3tau_min,State_over]=req_response()
%% required response:all values are in ms:

Z_step = 1 ; % 
Z_3tau_max = 4000 ; % 4 s to get 95% or less of 1 m
Z_3tau_min = 3000;
Z_over = 30; % 20% of the Z_step , overshooting

Phi_step = 0.0;
Phi_3tau_max = 800;
Phi_3tau_min = 700;
Phi_over = 20;

Theta_step = 0.0;
Theta_3tau_max = 800;
Theta_3tau_min = 700;
Theta_over = 30;

Psi_step = 0;
Psi_3tau_max = 800;% to get the max step 0.2 rad
Psi_3tau_min= 700;
Psi_over = 30;

% all in vectors:
State_step = [Z_step;Phi_step;Theta_step;Psi_step];
State_3tau_max = [Z_3tau_max;Phi_3tau_max;Theta_3tau_max;Psi_3tau_max];
State_3tau_min = [Z_3tau_min;Phi_3tau_min;Theta_3tau_min;Psi_3tau_min];
State_over = [Z_over;Phi_over;Theta_over;Psi_over];

end