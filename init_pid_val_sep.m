function init_pid_val=init_pid_val_sep(Copter_type)
%% PIDs paramters- initial values:
%fprintf('Calling Init values :\n');
if Copter_type == 1
    P_Z = 10;
    I_Z = 4;
    D_Z = 6;

    P_Phi = 12;
    I_Phi = 0.5;
    D_Phi = 0.7;

%     P_Theta = 12;
%     I_Theta = 0.05;
%     D_Theta = 0.7;
    
    P_Theta = 5;
    I_Theta = 0.05;
    D_Theta = 0.7;

%     P_Psi = 0.3;
%     I_Psi = 0.01;
%     D_Psi = 0.15;    
    P_Psi = 0.5;
    I_Psi = 0.001;
    D_Psi = 0.35;   
else
    if(Copter_type == 2)
            P_Z = 10;
            I_Z = 4;
            D_Z = 6;

            P_Phi = 6;
            I_Phi = 0.5;
            D_Phi = 0.4;

            P_Theta = 8;
            I_Theta = 0.5;
            D_Theta = 0.6;

            P_Psi = 0.4;
            I_Psi = 0.1;
            D_Psi = 0.2; 
    else
            P_Z = 10;
            I_Z = 4;
            D_Z = 6;

            P_Phi = 17;
            I_Phi = 1;
            D_Phi = 1;

            P_Theta = 18;
            I_Theta = 10;
            D_Theta = 1;

            P_Psi = 2;
            I_Psi = 0.05;
            D_Psi = 0.15;
    end
end


init_pid_val = [P_Z,I_Z,D_Z;
                P_Phi,I_Phi,D_Phi;
                P_Theta,I_Theta,D_Theta;
                P_Psi,I_Psi,D_Psi];


end