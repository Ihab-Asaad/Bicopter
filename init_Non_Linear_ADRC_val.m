function [parms,r,mult_obs,epsi]   = init_Non_Linear_ADRC_val()
mult_obs=10;
r = [10,5,10,10];
epsi = 1.5;
wcl = [3, 1,   4,   5]';
eta =   [1,   1,  1,  1]';
gamma = [1,   1,    1,    1]';
b = 0.5*[1,    1, 6,  6]';
parms = ones(4,4);
parms(:,1) = wcl;
parms(:,2) = eta;
parms(:,3) = gamma;
parms(:,4) = b;

end



