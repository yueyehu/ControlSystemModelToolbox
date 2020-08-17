function QuadcopterModelAda(block)
setup(block);

function setup(block)

  % Register the number of ports.
  %------
  block.NumInputPorts  = 5;
  %------
  block.NumOutputPorts = 18;
  
  % Set up the port properties to be inherited or dynamic.
  
  for i = 1:5; % These are the motor inputs
  block.InputPort(i).Dimensions        = 1;
  block.InputPort(i).DirectFeedthrough = false;
  block.InputPort(i).SamplingMode      = 'Sample';
  end
  %------

  for i = 1:18;
  block.OutputPort(i).Dimensions       = 1;
  block.OutputPort(i).SamplingMode     = 'Sample';
  end


  % Register the parameters.
  block.NumDialogPrms     = 6;
  
  % Set up the continuous states.
  block.NumContStates = 18;

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
     J   = block.DialogPrm(1).Data;
     R0  = block.DialogPrm(2).Data;
     m = block.DialogPrm(3).Data;
    

function InitializeConditions(block)
% Initialize 12 States

R0 = block.DialogPrm(2).Data;

for i=1:9
    block.OutputPort(i).Data = 0;
    block.ContStates.Data(i) = 0;
end
for i=1:3
    for j=1:3
        block.OutputPort(6+i*3+j).Data = R0(i,j);
        block.ContStates.Data(6+i*3+j) = R0(i,j);
    end
end

function Outputs(block)
for i = 1:18;
  block.OutputPort(i).Data = block.ContStates.Data(i);
end

function Derivatives(block)
J = block.DialogPrm(1).Data;
m = block.DialogPrm(3).Data;
time = block.InputPort(5).Data;
if time >1.5
    m = m*0.8;
    J = J*[0.9 0 0;0 0.9 0; 0 0 0.8];
end
g = 9.81;
% pos = block.ContStates.Data(1:3);
vel = block.ContStates.Data(4:6);
omega = block.ContStates.Data(7:9);
omega_hat = [0,-omega(3),omega(2);omega(3),0,-omega(1);-omega(2),omega(1),0];
for i=1:3
    R(i,:) = block.ContStates.Data(3*i+7:3*(i+3));
end

f = block.InputPort(1).Data;
for i=2:4
    u(i-1) = block.InputPort(i).Data;
end
d_pos = vel;
e3=[0;0;1];
d_vel = (m*g*e3-f*R*e3)/m;
d_omega = inv(J)*(u.'-omega_hat*J*omega);
d_R = R*omega_hat;

f = [d_pos(1),d_pos(2),d_pos(3),d_vel(1),d_vel(2),d_vel(3),d_omega(1),d_omega(2),d_omega(3),d_R(1,1),d_R(1,2),d_R(1,3),d_R(2,1),d_R(2,2),d_R(2,3),d_R(3,1),d_R(3,2),d_R(3,3)].';
  %This is the state derivative vector
block.Derivatives.Data = f;


%endfunction
