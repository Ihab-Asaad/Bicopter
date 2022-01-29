function init_pid_val=init_pid_val(Copter_type)
%% PIDs paramters- initial values:
%fprintf('Calling Init values :\n');
if Copter_type == 1
%     P_Z = 25;
%     I_Z = 10;
%     D_Z = 15;
% 
%     P_Phi = 1.2;
%     I_Phi = 0.2;
%     D_Phi = 0.3;
% 
%     P_Theta = 1.9;
%     I_Theta = 0.2;
%     D_Theta = 0.5;
% 
%     P_Psi = 1.2;
%     I_Psi = 0.3;
%     D_Psi = 0.3;    
    
%      P_Z = 25;
%      I_Z = 10;
%      D_Z = 15;
    
     P_Z = 4;
     I_Z = 0.1;
     D_Z = 3;
    
    P_Phi = 1.7;
    I_Phi = 0.2;
    D_Phi = 0.35;

    P_Theta = 1;
    I_Theta = 0.1;
    D_Theta = 0.4;

    P_Psi = 1.2;
    I_Psi = 0.3;
    D_Psi = 0.3;  
else
    if(Copter_type == 2)
        P_Z = 20;
        I_Z = 7;
        D_Z = 7.5;

        P_Phi = 5;
        I_Phi = 1;
        D_Phi = 2;

        P_Theta = 20;
        I_Theta = 10;
        D_Theta = 3;

        P_Psi = 0.3;
        I_Psi = 0.1;
        D_Psi = 0.07; 
    else
        P_Z = 20;
        I_Z = 7;
        D_Z = 7.5;

        P_Phi = 12;
        I_Phi = 3;
        D_Phi = 2;

        P_Theta = 20;
        I_Theta = 10;
        D_Theta = 3;

        P_Psi = 0.3;
        I_Psi = 0.1;
        D_Psi = 0.07; 
    end
end


init_pid_val = [P_Z,I_Z,D_Z;
                P_Phi,I_Phi,D_Phi;
                P_Theta,I_Theta,D_Theta;
                P_Psi,I_Psi,D_Psi];


end