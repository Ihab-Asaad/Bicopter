function init_pid_val=init_pid_val_sep()
%% PIDs paramters- initial values:
%fprintf('Calling Init values :\n');
    P_Z = 10;
    I_Z = 4;
    D_Z = 6;

%     P_Phi = 12;
%     I_Phi = 0.5;
%     D_Phi = 0.7;
%     P_Phi = 2.1;
%     I_Phi = 0.2;
%     D_Phi = 0.4;
    P_Phi = 3;
    I_Phi = 0.2;
    D_Phi = 0.5;

% %     P_Theta = 12;
% %     I_Theta = 0.05;
% %     D_Theta = 0.7;
    
%     P_Theta = 5;
%     I_Theta = 0.05;
%     D_Theta = 0.7;
%     P_Theta = 6;
%     I_Theta = 0.05;
%     D_Theta = 0.85;
    P_Theta = 5;
    I_Theta = 0.1;
    D_Theta = 0.7;

%     P_Psi = 0.3;
%     I_Psi = 0.01;
%     D_Psi = 0.15;    
%     P_Psi = 0.5;
%     I_Psi = 0.001;
%     D_Psi = 0.35;   
    P_Psi = 0.5;
    I_Psi = 0.01;
    D_Psi = 0.35;  

init_pid_val = [P_Z,I_Z,D_Z;
                P_Phi,I_Phi,D_Phi;
                P_Theta,I_Theta,D_Theta;
                P_Psi,I_Psi,D_Psi];


end