function [Copter] = init_Copter_sim(stop_time,step_size,sim_mode,tolerance,save_inputdata,save_outputdata,solver_type,solver)
%% Simulate dynamic system
%sim('Control_Bicopter',Simulation_Time)
%Copter.SimulationMode = 'accel';% run the simulation faster
Copter.SimulationMode = sim_mode; % run in normal mode
%Copter.SimulationMode = 'rapid-accelerator';
Copter.AbsTol         = tolerance;
Copter.SaveState      = save_inputdata;
Copter.StateSaveName  = 'xoutNew';
Copter.SaveOutput     = save_outputdata;
Copter.OutputSaveName = 'youtNew';% Position + Euler Angles
Copter.StartTime      = '0.0';
Copter.StopTime       = stop_time;
Copter.SolverType     = solver_type;
Cotper.Solver         = solver; % The mode of the solver
%Cotper.Solver         = 'VariableStepDiscrete';
%Cotper.Solver         = 'ode45';
Copter.FixedStep      = step_size;
%Copter.FixedStep      = 'auto';


end