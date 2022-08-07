function msfun_EMS_PMP(block)
%MSFUNTMPL_BASIC A Template for a Level-2 MATLAB S-Function
%   The MATLAB S-function is written as a MATLAB function with the
%   same name as the S-function. Replace 'msfuntmpl_basic' with the
%   name of your S-function.
%
%   It should be noted that the MATLAB S-function is very similar
%   to Level-2 C-Mex S-functions. You should be able to get more
%   information for each of the block methods by referring to the
%   documentation for C-Mex S-functions.
%
%   Copyright 2003-2010 The MathWorks, Inc.

%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

%endfunction

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C-Mex counterpart: mdlInitializeSizes
%%
function setup(block)

% Register number of ports
block.NumInputPorts  = 3;
block.NumOutputPorts = 5;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions        = 10;                                   % the inputs are demanded power and SOC
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;
block.InputPort(1).SamplingMode = 'Sample';

block.InputPort(2).Dimensions        = [1 1000];                                   % the inputs are demanded power and SOC
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = true;
block.InputPort(2).SamplingMode = 'Sample';

block.InputPort(3).Dimensions        = [1 1000];                                   % the inputs are demanded power and SOC
block.InputPort(3).DatatypeID  = 0;  % double
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = true;
block.InputPort(3).SamplingMode = 'Sample';

% Override output port properties
block.OutputPort(1).Dimensions       = 4;                                   % the outputs are APU power and battery power
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';

block.OutputPort(2).Dimensions       = [1 1000];                                   % the outputs are APU power and battery power
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';
block.OutputPort(2).SamplingMode = 'Sample';

block.OutputPort(3).Dimensions       = [1 1000];                                   % the outputs are APU power and battery power
block.OutputPort(3).DatatypeID  = 0; % double
block.OutputPort(3).Complexity  = 'Real';
block.OutputPort(3).SamplingMode = 'Sample';

block.OutputPort(4).Dimensions       = [49 1];                                   % the outputs are APU power and battery power
block.OutputPort(4).DatatypeID  = 0; % double
block.OutputPort(4).Complexity  = 'Real';
block.OutputPort(4).SamplingMode = 'Sample';

block.OutputPort(5).Dimensions       = [49 1];                                   % the outputs are APU power and battery power
block.OutputPort(5).DatatypeID  = 0; % double
block.OutputPort(5).Complexity  = 'Real';
block.OutputPort(5).SamplingMode = 'Sample';
% Register parameters
block.NumDialogPrms     = 16;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [-1 0];                                                  % the sampling time is 1s

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------

block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C-Mex counterpart: mdlSetWorkWidths
%%
function DoPostPropSetup(block)
%  block.NumDworks = 1;

%  block.Dwork(1).Name            = 'lambda';
%  block.Dwork(1).Dimensions      = 1;
%  block.Dwork(1).DatatypeID      = 0;      % double
%  block.Dwork(1).Complexity      = 'Real'; % real
%  block.Dwork(1).UsedAsDiscState = true;


%%
%% InitializeConditions:
%%   Functionality    : Called at the start of simulation and if it is
%%                      present in an enabled subsystem configured to reset
%%                      states, it will be called when the enabled subsystem
%%                      restarts execution to reset the states.
%%   Required         : No
%%   C-MEX counterpart: mdlInitializeConditions
%%
function InitializeConditions(block)

%end InitializeConditions


%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this
%%                      is the place to do it.
%%   Required         : No
%%   C-MEX counterpart: mdlStart
%%
function Start(block)

%block.Dwork(1).Data = block.DialogPrm(4).Data;

%endfunction

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C-MEX counterpart: mdlOutputs
%%
function Outputs(block)

%%非线性soc规划
time=block.InputPort(1).Data(1);%当前实际时间
edge=block.InputPort(1).Data(2);%当前的edge -- const
edge_last=block.InputPort(1).Data(3);
Qdmd_last=block.InputPort(1).Data(4);
soc_end=block.InputPort(1).Data(5);%当前soc的实际值
s_act=block.InputPort(1).Data(6);%当前参考位置/m
Pdmd = block.InputPort(1).Data(7);
SOCinterp_last=block.InputPort(1).Data(10);%当前SOC参考值
s_ref_last=block.InputPort(2).Data;%里程横轴参考
SOC_ref_last=block.InputPort(3).Data;%SOC纵轴参考

trafficspeed=block.DialogPrm(1).Data;
GradeEdge=block.DialogPrm(2).Data;
ratio=block.DialogPrm(3).Data;
PM=block.DialogPrm(4).Data;
Qnom=block.DialogPrm(5).Data;
regR=block.DialogPrm(6).Data;
RefSpdLmt=block.DialogPrm(14).Data;
distance_edge=block.DialogPrm(15).Data;
VelDis=block.DialogPrm(16).Data;

CDRoutput=zeros(49,1);
persistent s_goal;
if time<=1
    s_goal=0;
end
if  s_act>=s_goal 
    SOC_PLANNING;%输出a_ref和b_ref作为参考轨迹,-- 平均分段的SOC规划方法
    figure(1);plot(s_ref,SOC_ref);hold on;
    block.OutputPort(1).Data(3) = PMcon;
    block.OutputPort(4).Data = CDRoutput;
    block.OutputPort(5).Data = CDRMethod;
    s_goal=s_goal+5e3;
    flag=0;
else
    s_ref=s_ref_last;
    SOC_ref=SOC_ref_last;
    flag=1;
end

block.OutputPort(2).Data = s_ref;
block.OutputPort(3).Data = SOC_ref;
SOC_interp=interp1(s_ref,SOC_ref,s_act);
block.OutputPort(1).Data(2) = SOC_interp;
block.OutputPort(1).Data(4) = flag;
%%N-pmp功率分割
SOC = soc_end;%当前实际的SOC
Papu = block.InputPort(1).Data(8);
lambda = block.InputPort(1).Data(9);

ess_soc = block.DialogPrm(7).Data;
ess_voc = block.DialogPrm(8).Data;
ess_ro = block.DialogPrm(9).Data;
Ns = block.DialogPrm(10).Data;
Np = block.DialogPrm(11).Data;
K = block.DialogPrm(12).Data;
p2 = block.DialogPrm(13).Data;

NumericalPMP;
% AnalyticalPMP;
block.OutputPort(1).Data(1) = PapuCtrl;
%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlUpdate
%%
function Update(block)

%block.Dwork(1).Data = block.InputPort(1).Data;

%end Update

%%
%% Derivatives:
%%   Functionality    : Called to update derivatives of
%%                      continuous states during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlDerivatives
%%
function Derivatives(block)

%end Derivatives

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C-MEX counterpart: mdlTerminate
%%
function Terminate(block)

%end Terminate

