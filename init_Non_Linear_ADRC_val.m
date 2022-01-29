function [parms,r,mult_obs,epsi]   = init_Non_Linear_ADRC_val(Copter_type)
mult_obs=10;
if Copter_type==1
    mult_obs = 10;
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
    parms(:,4) = b
elseif Copter_type==2
    mult_obs=10;
    r = [7,0.5,7,7];
    epsi =0.7;
    wcl = [2, 1.8,   2,   1.8]';
    eta =   [0.001,   0.01,  0.001,  0.001]';
    gamma = [0.8,   0.5,    0.8,    0.5]';
    b = [10,    1, 10,  10]';
    parms = ones(4,4);
    parms(:,1) = wcl;
    parms(:,2) = eta;
    parms(:,3) = gamma;
    parms(:,4) = b;
else
        mult_obs=10;
        r = [7,0.5,7,7];
    epsi =0.7;
    wcl = [2, 1.8,   2,   1.8]';
    eta =   [0.001,   0.01,  0.001,  0.001]';
    gamma = [0.8,   0.5,    0.8,    0.5]';
    b = [10,    1, 10,  10]';
    parms = ones(4,4);
    parms(:,1) = wcl;
    parms(:,2) = eta;
    parms(:,3) = gamma;
    parms(:,4) = b;
    end
end

%%

%%



