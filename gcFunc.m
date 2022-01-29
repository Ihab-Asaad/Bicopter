function J = gcFunc(new_parms,tune,parms,file_name,Copter,size_step,parameterName)

ref = [1,0,0,0,0,0,0,0];
PP = parms;
sz = size(tune);
sz = max(sz);
if sz==1
    if (tune ==1)
    PP(tune,:)= new_parms;
    else
    PP(5-tune,:)= new_parms;
    end
else
    PP(tune,:)= reshape(new_parms,sz,[]);
end
assignin('base',parameterName,PP);
simOut = sim(file_name,Copter);
y = ref-simOut.state_space;
u = simOut.Control_inputs;
Q = diag([2,10,10,10,5,5,5,5]);
err_part = Q*y';
err = err_part*err_part';
err = sum(err(:));

u = u'*u;
u = sum(u(:));
u = 0;
J = size_step*(err+u);
J

