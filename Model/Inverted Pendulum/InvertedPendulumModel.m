function InvertedPendulumModel(block)
setup(block);

function setup(block)

  % Register the number of ports.
  %------
  block.NumInputPorts  = 1;
  %------
  block.NumOutputPorts = 4;
  
  % Set up the port properties to be inherited or dynamic.
  
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(1).SamplingMode      = 'Sample';
  
  for i = 1:4
  block.OutputPort(i).Dimensions       = 1;
  block.OutputPort(i).SamplingMode     = 'Sample';
  end

  % Register the parameters.
  block.NumDialogPrms     = 7;
  
  % Set up the continuous states.
  block.NumContStates = 4;

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
 m = block.DialogPrm(1).Data;
 M = block.DialogPrm(2).Data;
 L = block.DialogPrm(3).Data;
 Kc = block.DialogPrm(4).Data;
 Kp = block.DialogPrm(5).Data;
 x = block.DialogPrm(6).Data;
 theta = block.DialogPrm(7).Data;
    
function InitializeConditions(block)
% Initialize 6 States
for i=1:4
    block.OutputPort(i).Data = 0;
    block.ContStates.Data(i) = 0;
end
block.OutputPort(1).Data = block.DialogPrm(6).Data;
block.ContStates.Data(1) = block.DialogPrm(6).Data;
block.OutputPort(3).Data = block.DialogPrm(7).Data;
block.ContStates.Data(3) = block.DialogPrm(7).Data;

function Outputs(block)
for i = 1:4
  block.OutputPort(i).Data = block.ContStates.Data(i);
end

function Derivatives(block)
m = block.DialogPrm(1).Data;
M = block.DialogPrm(2).Data;
L = block.DialogPrm(3).Data;
Kc = block.DialogPrm(4).Data;
Kp = block.DialogPrm(5).Data;
g = 9.81;

% get the state of model
vel = block.ContStates.Data(2);
theta = block.ContStates.Data(3);
omega = block.ContStates.Data(4);

%get input
F = block.InputPort(1).Data;

M = [m+M m*L*cos(theta);
     m*L*cos(theta) 4/3*m*L^2];
C = [0 -m*L*sin(theta)*omega;
     0 0];
G = [0;
     -m*g*L*sin(theta)];
T = [F-Kc*vel;
     -Kp*omega];
d_v = M\(T - C*[vel;omega] - G);

d_state = [vel,d_v(1),omega,d_v(2)].';
%This is the state derivative vector
block.Derivatives.Data = d_state;
%endfunction
