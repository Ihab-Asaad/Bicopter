function [b_sep,wc_sep,wo_sep,eps_sep] = get_adrc_sep()
ga = 8;
wc_sep = [ 5.54   9   6    3];
eps_sep = [0.7 0.3 0.3 0.35];
%b_sep =[  0.8   4    4   8];
b_sep =[  2   7    6   7];
wo_sep = ga*wc_sep;
end