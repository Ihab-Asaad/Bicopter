function init_STSC=init_STSC_val(Copter_type)
%% STSC paramters- initial values:
% STSC Super-Twisting sliding mode controller
% u = alpha*sqrt(sigma)*sign(sigma)+beta*integral(sign(sigma))
%fprintf('Calling Init values :\n');
if Copter_type == 1
    %% sigma = lambda*e + e_dot
%     STphi_lambda =8;
%     STphi_alpha = 0.5;
%     STphi_beta =  0.05;
    STphi_lambda = 10;
    STphi_alpha = 0.8;
    STphi_beta =  0.05;
%     STtheta_lambda =11;
%     STtheta_alpha = 0.3;
%     STtheta_beta =  0.1;
    
    STtheta_lambda =7;
    STtheta_alpha = 2;
    STtheta_beta =  0.02;
    
    STpsi_lambda =4;
    %STpsi_alpha =0.1;
    STpsi_alpha =0.2;
%     STpsi_beta =  0.01;
    STpsi_beta =  0.1;
    
    STz_lambda =2;
    STz_alpha = 7;
%     STz_beta =  0.8;
    STz_beta =  1.6;


elseif Copter_type == 2
    STphi_lambda =4;
    STphi_alpha = 0.1;
    STphi_beta =  0.05;
    
    STtheta_lambda =4;
    STtheta_alpha = 0.1;
    STtheta_beta =  0.05;
    
    STpsi_lambda =2;
    STpsi_alpha =0.01;
    STpsi_beta =  0.005;
    
    STz_lambda =2;
    STz_alpha = 10;
    STz_beta =  2;
else
    STphi_lambda =12;
    STphi_alpha = 1;
    STphi_beta =  0.3;
    
    STtheta_lambda =10;
    STtheta_alpha = 2;
    STtheta_beta =  0.3;
    
    STpsi_lambda =2.5;
    STpsi_alpha =0.11;
    STpsi_beta =  0.15;
    
    STz_lambda =2;
    STz_alpha = 7;
    STz_beta =  0.8;
end


init_STSC = [STz_lambda,STz_alpha,STz_beta;
                STphi_lambda,STphi_alpha,STphi_beta;
                STtheta_lambda,STtheta_alpha,STtheta_beta;
                STpsi_lambda,STpsi_alpha,STpsi_beta];


end