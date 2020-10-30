function QuadrotorModel(block)
setup(block);

function setup(block)

  % Register the number of ports.
  %------
  block.NumInputPorts  = 5;
  %------
  block.NumOutputPorts = 6;
  
  % Set up the port properties to be inherited or dynamic.
  
  for i = 1:4 % These are the motor inputs
  block.InputPort(i).Dimensions        = 1;
  block.InputPort(i).DirectFeedthrough = false;
  block.InputPort(i).SamplingMode      = 'Sample';
  end
  
  block.InputPort(5).Dimensions        = 6;
  block.InputPort(5).DirectFeedthrough = false;
  block.InputPort(5).SamplingMode      = 'Sample';
  
  for i = 1:6
  block.OutputPort(i).Dimensions       = 1;
  block.OutputPort(i).SamplingMode     = 'Sample';
  end

  % Register the parameters.
  block.NumDialogPrms     = 2;
  
  % Set up the continuous states.
  block.NumContStates = 12;

  block.SampleTimes = [0 0];
  
  block.SetAccelRunOnTLC(false);
  
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('CheckParameters', @CheckPrms);

  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  
  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Derivatives', @Derivatives);
  
% -------------------------------------------------------------------
% The local functions below are provided to illustrate how you may implement
% the various block methods listed above.
% -------------------------------------------------------------------

 function CheckPrms(block)
 I = block.DialogPrm(1).Data;
 m = block.DialogPrm(2).Data;
    

function InitializeConditions(block)
% Initialize 6 States
for i=1:6
    block.OutputPort(i).Data = 0;
end
for i=1:12
    block.ContStates.Data(i) = 0;
end

function Outputs(block)
for i = 1:3
  block.OutputPort(i).Data = block.ContStates.Data(i);
end
for i = 4:6
  block.OutputPort(i).Data = block.ContStates.Data(i + 3);
end

function Derivatives(block)
I = block.DialogPrm(1).Data;
m = block.DialogPrm(2).Data;
g = 9.81;

% get the state of model
vel = block.ContStates.Data(4:6);
euler = block.ContStates.Data(7:9);
omega = block.ContStates.Data(10:12);
phi = euler(1); theta = euler(2); psi = euler(3);
R = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) - sin(phi)*sin(psi);
     cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
     -sin(theta),         sin(phi)*cos(theta),                              cos(phi)*cos(theta)];
R_oa = [1, sin(phi)*sin(theta)/cos(theta), cos(phi)*sin(theta)/cos(theta);
        0, cos(phi),                       -sin(phi);
        0, sin(phi)/cos(theta),            cos(phi)/cos(theta)];
omega_hat = [0,-omega(3),omega(2);omega(3),0,-omega(1);-omega(2),omega(1),0];
e3 = [0;0;1];

%get input
F_l = block.InputPort(1).Data;
for i = 2:4
    Tau(i - 1) = block.InputPort(i).Data;
end

% Get Drag coefficients
D_v = block.InputPort(5).Data(1:3);
D_omega = block.InputPort(5).Data(4:6);

% model
d_pos = vel;
d_vel = (m * g * e3 - F_l * R * e3 - D_v .* vel)/m;
d_euler = R_oa * omega;
d_omega = I\(Tau.'-omega_hat * I * omega - D_omega .* omega);

d_state = [d_pos(1),d_pos(2),d_pos(3),d_vel(1),d_vel(2),d_vel(3),d_euler(1),d_euler(2),d_euler(3), d_omega(1),d_omega(2),d_omega(3)].';
%This is the state derivative vector
block.Derivatives.Data = d_state;
%endfunction
