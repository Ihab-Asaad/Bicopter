function [Para_min,Para_max] = parameters_min_max()

% PID _Z : 
P_Z_min = 20;
P_Z_max = 30;

I_Z_min = 5;
I_Z_max = 15;

D_Z_min = 10;
D_Z_max = 20;

% PID _Phi : 
P_Phi_min = 0.9;
P_Phi_max = 1.4;

I_Phi_min = 0.1;
I_Phi_max = 0.5;

D_Phi_min = 0.1;
D_Phi_max = 0.6;

% PID _Theta : 
P_Theta_min = 1.6;
P_Theta_max = 2.3;

I_Theta_min = 0.01;
I_Theta_max = 0.4;

D_Theta_min = 0.2;
D_Theta_max = 0.8;

% PID _Psi : 
P_Psi_min = 0.9;
P_Psi_max = 1.5;

I_Psi_min = 0.1;
I_Psi_max = 0.6;

D_Psi_min = 0.1;
D_Psi_max = 0.6;

% all in one vector:
Para_min = [P_Z_min,I_Z_min,D_Z_min;
            P_Phi_min,I_Phi_min,D_Phi_min;
            P_Theta_min,I_Theta_min,D_Theta_min;
            P_Psi_min,I_Psi_min,D_Psi_min];

Para_max = [P_Z_max,I_Z_max,D_Z_max;
            P_Phi_max,I_Phi_max,D_Phi_max;
            P_Theta_max,I_Theta_max,D_Theta_max;
            P_Psi_max,I_Psi_max,D_Psi_max;];


end