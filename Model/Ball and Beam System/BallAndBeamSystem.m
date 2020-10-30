function BallAndBeamSystem(block)
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
  block.NumDialogPrms     = 5;
  
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
  R = block.DialogPrm(2).Data;
  J = block.DialogPrm(3).Data;
  M = block.DialogPrm(4).Data;
  L = block.DialogPrm(5).Data;
    

function InitializeConditions(block)
% Initialize 6 States
for i=1:4
    block.OutputPort(i).Data = 0;
    block.ContStates.Data(i) = 0;
end

function Outputs(block)
 L = block.DialogPrm(5).Data;
 for i = 1:4
  block.OutputPort(i).Data = block.ContStates.Data(i);
 end

function Derivatives(block)
 m = block.DialogPrm(1).Data;
 R = block.DialogPrm(2).Data;
 J = block.DialogPrm(3).Data;
 M = block.DialogPrm(4).Data;
 L = block.DialogPrm(5).Data;
 g = 9.81;

% get the state of model
pos = block.ContStates.Data(1);
vel = block.ContStates.Data(2);
alpha = block.ContStates.Data(3);
omega = block.ContStates.Data(4);

%get input
tau = block.InputPort(1).Data;


% model
d_pos = vel;
d_vel = (m*g*sin(alpha)-m*(L-pos)*omega^2)/(m+J/R^2);
d_alpha = omega;
d_omega = (tau+2*m*(L-pos)*vel*omega-(m*g*(L-pos)+0.5*M*g*L)*cos(alpha))/(m*(L-pos)^2+1/3*M*L^2);
    
d_state = [d_pos, d_vel, d_alpha, d_omega].';
%This is the state derivative vector
block.Derivatives.Data = d_state;
%endfunction
