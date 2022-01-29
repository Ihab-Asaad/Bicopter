function J = gcFunc_2(new_parms,tune,parms,file_name,Copter,size_step,parameterName,isADRC)
    function err = getcost(time,desired_time,delta)
        if time<desired_time
            err = 0;
        else
            err = (time-desired_time)^2;
        end
    end
if isADRC
    try
        %[b_sep,wc_sep,wo_sep,eps_sep] = get_ADRC_parameters(new_parms);
        [K,L,b,C] = get_ADRC_parameters(new_parms);
    catch ME
        disp('Error was found')
        J = 1e10;
        return
    end
%     assignin('base','b_sep',b_sep);
%     assignin('base','wc_sep',wc_sep);
%     assignin('base','wo_sep',wo_sep);
%     assignin('base','eps_sep',eps_sep);
    assignin('base','K',K);
    assignin('base','L',L);
    assignin('base','b',b);
    assignin('base','C',C);
    
else
    PP = parms;
    sz = size(tune);
    sz = max(sz);
    if sz==1
        PP(tune,:)= new_parms;

    else
       for  i=1:sz
        PP(tune(i),:)= new_parms(1+3*(i-1):3+3*(i-1));
       end

    end
    
    assignin('base',parameterName,PP);
end

simOut = sim(file_name,Copter);
phi = simOut.Error(:,1);
theta = simOut.Error(:,3);
psi = simOut.Error(:,4);
z = simOut.Error(:,2);
t = simOut.tout;
phi_res = stepinfo(phi,t);
theta_res = stepinfo(theta,t);
psi_res = stepinfo(psi,t);
z_res = stepinfo(z,t);
%stepResults2 = stepinfo(simOut.Euler(:,2),simOut.tout);
t_psi = psi_res.SettlingTime;
t_theta = theta_res.SettlingTime;
t_phi = phi_res.SettlingTime;
t_z = z_res.SettlingTime;
err_phi = sum(abs(phi));
err_theta =  sum(abs(theta));
err_psi =  sum(abs(psi));
err_z = sum(abs(z));
J = 0;
des_z = 4;
des_phi = 0.1;
des_theta = 0.1;
des_psi = 0.15;
if ismember(1,tune)
   % J = J+getcost(t_z,des_z,0.1)^2+err_z;
     J = J+err_z;
end
if ismember(2,tune)
    %J = J+getcost(t_phi,des_phi,0.1)^2+err_phi;
    J = J+err_phi;
end
if ismember(3,tune)
%     J = J+getcost(t_theta,des_theta,0.1)^2+err_theta;
    J = J+err_theta;

end
if ismember(4,tune)
%     J = J+getcost(t_psi,des_psi,0.1)^2+err_psi;
    J = J + err_psi;

end
%PP
if isADRC
    new_parms
else
    PP
end
t_phi
t_theta
t_psi
t_z
J
end

% u = simOut.Control_inputs;
% Q = diag([2,10,10,10,5,5,5,5]);
% err_part = Q*y';
% err = err_part*err_part';
% err = sum(err(:));
% 
% u = u'*u;
% u = sum(u(:));
% u = 0;
% J = size_step*(err+u);
% J

