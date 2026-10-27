function d_error = controlNMPC(states, inputs)

    % states: lateral error and heading error
    % inputs: absolute wheel speeds and curvature
    
    % if no existing states or inputs then set up NMPC
    if nargin == 0
        if coder.target('MATLAB')
            setupNMPC();
        end
        return
    end
    
    % parameters
    r_w = 0.0338;
    d = 0.108;
    
    e_lat = states(1);
    e_psi = states(2);
    m1_rate = inputs(1);
    m2_rate = inputs(2);
    kappa = inputs(3);
    
    % forward velocity and angular velocity
    v = (r_w/2)*(m1_rate + m2_rate);
    omega = (r_w/d)*(m2_rate - m1_rate);
    % speed on path
    s_dot = v*cos(e_psi)/max(1 -(kappa*e_lat), 0.1);
    
    % Error rates
    de_lat = v*sin(e_psi);
    de_psi = omega - (kappa*s_dot);
    
    d_error = [de_lat; de_psi];

end

% Set up NMPC if it hasn't yet been set up 

function setupNMPC()

    % NMPC object with 2 states and 2 outputs 
    nlobj = nlmpc(2, 2, 'MV', [1 2], 'MD', 3);

    % Model set up 
    nlobj.Model.StateFcn = 'controlNMPC';
    nlobj.Model.IsContinuousTime = true;

    % sample time and horizons
    nlobj.Ts = 0.02;
    nlobj.PredictionHorizon = 25;
    nlobj.ControlHorizon = 2;

    % constraints

    % absolute wheel speeds (hard)
    dphi_limit = 9.72;
    for i = 1:2
        nlobj.MV(i).Min = -dphi_limit;
        nlobj.MV(i).Max =  dphi_limit;
    end

    % lateral error = half track width - half robot width (soft)
    nlobj.OV(1).Min = -0.235;
    nlobj.OV(1).Max =  0.235;
    nlobj.OV(1).MinECR = 1;
    nlobj.OV(1).MaxECR = 1;

    % heading error = 30 degrees (soft)
    nlobj.OV(2).Min = -deg2rad(30);
    nlobj.OV(2).Max =  deg2rad(30);
    nlobj.OV(2).MinECR = 1;
    nlobj.OV(2).MaxECR = 1;

    % same global slack penalty as LMPC
    nlobj.Weights.ECR = 1e5;

    % weights
    nlobj.Weights.OutputVariables = [0.7, 0.5];    % [e_lat, e_psi]
    nlobj.Weights.ManipulatedVariables = [0.05, 0.05]; % pull to MV target
    nlobj.Weights.ManipulatedVariablesRate  = [0.1, 0.1];

    % validate at 0.3m/s 
    u0 = (0.3/0.0338) * [1; 1];
    validateFcns(nlobj, [0; 0], u0, 0);

    % saving object to workspace for nmpc block
    save('nmpc_robot.mat', 'nlobj');

    % click on and import the nmpc_robot.mat file before running the NMPC in Simulink

end
