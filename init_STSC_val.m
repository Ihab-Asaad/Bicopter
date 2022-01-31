function init_STSC=init_STSC_val()
%% STSC paramters- initial values:
% STSC Super-Twisting sliding mode controller
% u = alpha*sqrt(sigma)*sign(sigma)+beta*integral(sign(sigma))
%fprintf('Calling Init values :\n');

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


init_STSC = [STz_lambda,STz_alpha,STz_beta;
                STphi_lambda,STphi_alpha,STphi_beta;
                STtheta_lambda,STtheta_alpha,STtheta_beta;
                STpsi_lambda,STpsi_alpha,STpsi_beta];


end