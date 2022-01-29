%function [K,L,b,C]  = get_ADRC_parameters(gains)
function [b_sep,wc_sep,wo_sep,eps_sep]  = get_ADRC_parameters(gains)
   %% defineing parameters with regrading response
ga = 5;
% wc_phi = gains(1);
% wc_theta = gains(2);
% wc_z = 1;
% wc_psi = 3;
wc_sep = gains(1:4);
eps_sep = 0.7*[1 1 1 1];
b_sep =gains(5:8);
wo_sep = ga*wc_sep;


% w1 = 1.2;
%  w2 = 1;
%  w3 = 1;
%  w4 = 1.5;

% w1 = gains(1);
% w2 = gains(2);
% w3 = gains(3);
% w4 = gains(4);
% %w0 = [w1 w2 w3 w4 w1 w2 w3 w4 w1 w2 w3 w4];
% % w0 = 1.8*ones(1,12);
% %w0 = 1.8*ones(1,12);
% w0 = [-w1 -w2 -w3 -w4 -3*w1 -3*w2 -3*w3 -3*w4]
% b = [3,1,3,5];
% ga1 = 0.1;
% ga2 = 0.1;
% ga = 10
% %% calculating K
% A1 = [zeros(4,4),eye(4);zeros(4,8)];
% B1 = [zeros(4,4);diag(b)];
% C1 = [eye(4),zeros(4,4)];
% Q = ga1*C1'*C1;
% %Q = [eye(4),zeros(4,4);zeros(4,4),2*eye(4)];
% R = ga1* eye(4);
% Aw = A1 +eye(8).*w0(1:8)';
% %Aw = A1 +eye(8)*w0;
% [K,S,P] = lqr(Aw,B1,Q,R);
% P
% K = place(A1,B1,w0)
% %% calculating L
% B = [zeros(4,4);diag(b);zeros(4,4)];
% C = [eye(4),zeros(4,8)];
% A = [zeros(4,4),eye(4),zeros(4,4);zeros(4,4),zeros(4,4),eye(4);zeros(4,12)];
% Q0 = C'*C;
% R0 = ga2*eye(4);
% A0 = A + ga*eye(12).*w0';
% %A0 = A + ga*w0;
% L = lqr(A0',C',Q0,R0)';
% 
end
