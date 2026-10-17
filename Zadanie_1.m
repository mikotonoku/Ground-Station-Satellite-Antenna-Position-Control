clearvars;
clc;
close all;

%%  T-36001 DC SERVO MOTOR PARAMETERS

Ra = 13.3;              % Armature resistance [Ohm]
La = 29;                % Armature inductance [H]

Jm = 51.386;            % Rotor inertia
Ja = 3.5946;            % Load inertia

Bm = 13.558;            % Viscous friction coefficient

Kt = 138;               % Torque constant
Kb = 187;               % Back-EMF constant

Deg = 150;              % Reference position [deg]
Ref = Deg*pi/180;       % Reference position [rad]

%%  DC MOTOR TRANSFER FUNCTION

num = Kt;

den = [ ...
    La*Jm, ...
    Ra*Jm + La*Bm, ...
    Ra*Bm + Kt*Kb, ...
    0 ...
];

G = tf(num, den);

fprintf('DC SERVO MOTOR\n');

fprintf('\nPlant G(s) = theta(s)/Va(s):\n');
disp(G);

%%  PID CONTROLLER

% Desired crossover frequency
wc = 0.5;       % [rad/s]

% Automatic PID tuning
C_pid = pidtune(G, 'PID', wc);

Kp = C_pid.Kp;
Ki = C_pid.Ki;
Kd = C_pid.Kd;

fprintf('PID CONTROLLER\n');

fprintf('Kp = %.6f\n', Kp);
fprintf('Ki = %.6f\n', Ki);
fprintf('Kd = %.6f\n', Kd);

A = [ ...
    -Ra/La,  -Kb/La,   0;
     Kt/Jm,  -Bm/Jm,   0;
     0,       1,       0 ...
];

B = [ ...
    1/La;
    0;
    0 ...
];

% Output = angular position in degrees
C = [0, 0, 180/pi];

D = 0;

sys_ss = ss(A, B, C, D);

%%  LQR CONTROLLER

Q = 1e6 * eye(3);n 
R = 10;

K_lqr = lqr(A, B, Q, R);

fprintf('LQR CONTROLLER\n');

fprintf('K_lqr = [%.4f  %.4f  %.4f]\n', K_lqr);


%% Fuzzy-PI

Ge = 1/Ref;
Gde = 0.8;
GKp = 130;
GKi = 20;

fprintf('FUZZY PI CONTROLLER\n');

fprintf('Reference = %.2f deg = %.6f rad\n', Deg, Ref);

fprintf('\nInput scaling:\n');
fprintf('Ge  = %.6f\n', Ge);
fprintf('Gde = %.6f\n', Gde);

fprintf('\nOutput scaling:\n');
fprintf('GKp = %.6f\n', GKp);
fprintf('GKi = %.6f\n', GKi);


%  LOAD FUZZY INFERENCE SYSTEM

if isfile('Fuzzy_PI.fis')

    Fuzzy_PI = readfis('Fuzzy_PI.fis');

    fprintf('\nFuzzy_PI.fis successfully loaded.\n');

else

    warning(['Fuzzy_PI.fis was not found. ', ...
             'Export the FIS from Fuzzy Logic Designer ', ...
             'and save it as Fuzzy_PI.fis.']);

end

%  FUZZY PI EQUATIONS
%
% Position error:
%
%       e(t) = Ref - theta(t)
%
%
% Scaled error:
%
%       e_fuzzy = Ge * e
%
%
% Change of error:
%
%       de/dt
%
%
% Scaled change of error:
%
%       de_fuzzy = Gde * de/dt
%
%
% Fuzzy Logic Controller:
%
%       [Kp_fuzzy, Ki_fuzzy]
%              =
%       FIS(e_fuzzy, de_fuzzy)
%
%
% Actual controller gains:
%
%       Kp(t) = GKp * Kp_fuzzy
%
%       Ki(t) = GKi * Ki_fuzzy
%
%
% Fuzzy PI control:
%
%                       /
%       u(t) = Kp(t)e + | Ki(t)e dt
%                       /
%
% ==========================================================