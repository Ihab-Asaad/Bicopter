function [K,L,b,C] = init_ADRC_val(Copter_type)
if Copter_type==1
    %% defineing parameters with regrading response
% w0 = 1.8;
% b = [13,1,10,10];
% ga1 = 1;
% ga2 = 1;
% ga = 5;
%w0 = 1.8;
 w1 = 2.2;
 w2 = 1;
 w3 = 2.2;
 w4 = 1.3;
w0 = [w1 w2 w3 w4 w1 w2 w3 w4 w1 w2 w3 w4];
%w0 = 1.8*ones(1,12);
b = [5,1,5,4];
ga1 = 0.5;
ga2 = 0.5;
ga = 8
%% calculating K
A1 = [zeros(4,4),eye(4);zeros(4,8)];
B1 = [zeros(4,4);diag(b)];
C1 = [eye(4),zeros(4,4)];
Q = C1'*C1;
%Q = [eye(4),zeros(4,4);zeros(4,4),2*eye(4)];
R = ga1* eye(4);
Aw = A1 +eye(8).*w0(1:8)';
%Aw = A1 +eye(8)*w0;
[K,S,P] = lqr(Aw,B1,Q,R);
w00 = [-w1 -w2 -w3 -w4 -w1 -w2 -w3 -w4 -5*w1 -5*w2 -5*w3 -5*w4];
K = place(A1,B1,w00(1:8));
%K(:,5:8) = K(:,5:8)/3;
%% calculating L
B = [zeros(4,4);diag(b);zeros(4,4)];
C = [eye(4),zeros(4,8)];
A = [zeros(4,4),eye(4),zeros(4,4);zeros(4,4),zeros(4,4),eye(4);zeros(4,12)];
Q0 = C'*C;
R0 = ga2*eye(4);
A0 = A + ga*eye(12).*w0';
%A0 = A + ga*w00;
% K = place(A1,B1,[-2*w1 -w2 -w3 -w4 -5*w1 -3*w2 -3*w3 -3*w4])

% L = lqr(A0',C',Q0,R0)';
L = place(A',C',8*w00)';
elseif Copter_type==2
    %% defineing parameters with regrading response
w0 = 1.8;
b = [13,1,10,10];
ga1 = 1;
ga2 = 1;
ga = 5;
%% calculating K
A1 = [zeros(4,4),eye(4);zeros(4,8)];
B1 = [zeros(4,4);diag(b)];
C1 = [eye(4),zeros(4,4)];
Q = C1'*C1;
R = ga1* eye(4);
Aw = A1 +w0*eye(8);
K = lqr(Aw,B1,Q,R);
%% calculating L
B = [zeros(4,4);diag(b);zeros(4,4)];
C = [eye(4),zeros(4,8)];
A = [zeros(4,4),eye(4),zeros(4,4);zeros(4,4),zeros(4,4),eye(4);zeros(4,12)];
Q0 = C'*C;
R0 = ga2*eye(4);
A0 = A +ga*w0*eye(12);
L = lqr(A0',C',Q0,R0)';

    else
    %% defineing parameters with regrading response
% w0 = 1.8;
% b = [13,1,10,10];
% ga1 = 1;
% ga2 = 1;
% ga = 5;

w0 =5*[1 1 1 1 1 1 1 1 1 1 1 1];
b = [4,1,4,10];
ga1 = 1;
ga2 = 1;
ga = 6
%% calculating K
A1 = [zeros(4,4),eye(4);zeros(4,8)];
B1 = [zeros(4,4);diag(b)];
C1 = [eye(4),zeros(4,4)];
Q = C1'*C1;
R = ga1* eye(4);
Aw = A1 +eye(8).*w0(1:8)';
K = lqr(Aw,B1,Q,R);
%% calculating L
B = [zeros(4,4);diag(b);zeros(4,4)];
C = [eye(4),zeros(4,8)];
A = [zeros(4,4),eye(4),zeros(4,4);zeros(4,4),zeros(4,4),eye(4);zeros(4,12)];
Q0 = C'*C;
R0 = ga2*eye(4);
A0 = A + ga*eye(12).*w0';
L = lqr(A0',C',Q0,R0)';

end     
end
