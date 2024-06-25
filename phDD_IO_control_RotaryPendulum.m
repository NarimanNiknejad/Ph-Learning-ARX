%% Cleaning

clc;
clear;
close all;

%% Initializing the Parameters

N = 20;  %number of collected data
N_s = 60; % number of steps for simulation
N_s_1 = 7; %number of steps of simulation without stabilization
N_s_2 = N_s - N_s_1; %number of steps of simulation with stabilization

T = 6; % burn-in period for data collection
L = 2;  %degree of the system

deltaT = 0.01; %Discretization step size


nOutputs = 2;    %dimension of the output vector
nInputs = 1;    %dimension of the input vector

q = nOutputs + nInputs;

epsilon = 0.007; %uncertain physics information

NoiseEnergy = (1e-2)*deltaT^4; % based on the characteristics of the system 


%% Nominal system parameters

Ln_r = 0.752;
Ln_p = 0.241;
mn_p = 1.17;
Jn_r = 2.0*10^(-2);
Cn_r = 0.47;
gn = 10;


P_n1 = [-2+Cn_r*deltaT/Jn_r, 0;
        Cn_r*Ln_r*deltaT/(Jn_r*Ln_p), -2];

Q_n1 = [0;
        0];

P_n0 = [1-(Cn_r*deltaT^2)/Jn_r, -(Ln_r*deltaT^2*gn*mn_p)/Jn_r;
        -Cn_r*Ln_r*deltaT/(Jn_r*Ln_p), 1-(gn*mn_p*Ln_r^2*deltaT^2)/(Jn_r*Ln_p)];

Q_n0 = [deltaT^2/Jn_r;
        (Ln_r*deltaT^2)/(Jn_r*Ln_p)];

R_n = [-Q_n0, P_n0,-Q_n1, P_n1];

%% Actual System Parameters

Ls_r = 0.75;
Ls_p = 0.239;
ms_p = 1.2;
Js_r = 1.98*10^(-2);
Cs_r = 0.46;
gs = 9.81;



P_s1 = [-2+Cs_r*deltaT/Js_r, 0;
        Cs_r*Ls_r*deltaT/(Js_r*Ls_p), -2];

Q_s1 = [0;
        0];

P_s0 = [1-(Cs_r*deltaT^2)/Js_r, -(Ls_r*deltaT^2*gs*ms_p)/Js_r;
        -Cs_r*Ls_r*deltaT/(Js_r*Ls_p), 1-(gs*ms_p*Ls_r^2*deltaT^2)/(Js_r*Ls_p)];

Q_s0 = [deltaT^2/Js_r;
        (Ls_r*deltaT^2)/(Js_r*Ls_p)];

R_s = [-Q_s0, P_s0,-Q_s1, P_s1];

epsilon_s = norm(R_s-R_n,2);
% disp(Q_s0);

% disp(epsilon_s); % Take epsilon larger than this value by some buffer

%% Ph-informed set 
 
O_phys = [epsilon^2*eye(nOutputs)-R_n*R_n',R_n;
    R_n', -eye(q*L)];
O_phys_projected = [ [zeros(nOutputs, q*(L-1)), zeros(nOutputs,nInputs),-eye(nOutputs)], zeros(nOutputs,q*L); zeros(q*L,q*L), eye(q*L) ]'*O_phys*[ [zeros(nOutputs, q*(L-1)), zeros(nOutputs,nInputs),-eye(nOutputs)], zeros(nOutputs,q*L); zeros(q*L,q*L), eye(q*L) ]; %dimension is 2qL by 2qL


%% Data collection from the actual system 

%First a bounded energy noise signal is generated

signals = randn(nOutputs, N+T); % Normally distributed random numbers (2-by-N matrix)
currentEnergy = sum(sum(signals.^2)); % Sum of squared values of all elements
scalingFactor = sqrt(NoiseEnergy / currentEnergy);
noise = signals * scalingFactor;

% Display the result
% disp(noise);

% Verify the total energy of the scaled signals
scaledEnergy = sum(sum(noise.^2));
% disp(['Desired energy: ', num2str(NoiseEnergy)]);
% disp(['Scaled signals total energy: ', num2str(scaledEnergy)]);

% data collection here: 


Y = zeros(nOutputs,N+T+2); 
Y(:,1) = [0.1,0.1]';
Y(:,2) = [0.11,0.09]';

U = -0.05 + 0.1*rand(nInputs,N+T+1);

for kk = 1:N+T
    Y(:,kk+2) = -R_s(:,1:3)*[U(:,kk);Y(:,kk)] - R_s(:,4:end)*[U(:,kk+1);Y(:,kk+1)]+noise(:,kk);
end

% removing the burn-in values
Y_1 = Y(:,T+1:end-1);
U_1 = U(:,T+1:end-1);
% disp(Y_1);

[H_1, H_2] = modifiedHankel(U_1, Y_1, L); % forming the modified Hankel

%% Forming the data-conformity set

O_n = [NoiseEnergy*eye(nOutputs), zeros(nOutputs,N-L+1);
        zeros(nOutputs,N-L+1)', -eye(N-L+1)];

O_data = [eye(nOutputs),H_2;zeros(q*L,nOutputs),H_1]*O_n*[eye(nOutputs),H_2;zeros(q*L,nOutputs),H_1]'; %dimension is qL + p by qL + p

O_data_projected = [ [zeros(nOutputs, q*(L-1)), zeros(nOutputs,nInputs),-eye(nOutputs)], zeros(nOutputs,q*L); zeros(q*L,q*L), eye(q*L) ]'*O_data*[ [zeros(nOutputs, q*(L-1)), zeros(nOutputs,nInputs),-eye(nOutputs)], zeros(nOutputs,q*L); zeros(q*L,q*L), eye(q*L) ]; %dimension is 2qL by 2qL

% disp(O_data)



%% optimization problem (finding stabilizng controller) both phys and data 


psi = sdpvar(q*L,q*L,'symmetric');
G = sdpvar(nInputs, q*L);
I_tilde = [zeros(q*(L-1),q), eye(q*(L-1))];

tau_dt = sdpvar(1,1);
tau_ph = sdpvar(1,1);
gama = sdpvar(1,1);

miu_dt =sdpvar(1,1);
miu_ph =sdpvar(1,1);


constraint_1 =  miu_dt * O_data_projected + miu_ph * O_phys_projected>= 0.0;
constraint_2 = [psi, -[I_tilde*psi;G;zeros(nOutputs,q*L)],[I_tilde*psi;G;zeros(nOutputs,q*L)];
                -[I_tilde*psi;G;zeros(nOutputs,q*L)]', -psi,zeros(q*L);
                [I_tilde*psi;G;zeros(nOutputs,q*L)]',zeros(q*L),psi]   - tau_ph*[O_phys_projected,zeros(2*q*L,q*L);
                                                                zeros(q*L,2*q*L),zeros(q*L,q*L)] -tau_dt*[O_data_projected,zeros(2*q*L,q*L);
                                                                                                  zeros(q*L,2*q*L),zeros(q*L,q*L)] >=0;

% constraint_2 = [psi, -[I_tilde*psi;G;zeros(nOutputs,q*L)],[I_tilde*psi;G;zeros(nOutputs,q*L)];
%                 -[I_tilde*psi;G;zeros(nOutputs,q*L)]', -psi,zeros(q*L);
%                 [I_tilde*psi;G;zeros(nOutputs,q*L)]',zeros(q*L),psi]   -tau_dt*[O_data_projected,zeros(2*q*L,q*L);
%                                                                                                   zeros(q*L,2*q*L),zeros(q*L,q*L)] >=0;

constraint_3 = tau_ph>=0.0001;
constraint_4 = tau_dt>=0.0001;

constraints =  constraint_1+constraint_2 + constraint_3 +constraint_4;
cost =0;

ops = sdpsettings('solver','mosek','verbose',0);
diagnostics = optimize(constraints, cost, ops);

if(diagnostics.problem == 0)

    psi_v = value(psi);
    G_v = value(G);
    tau_ph_v = value(tau_ph);
    tau_dt_v = value(tau_dt);


else

    fprintf('Error! Solution is infeasible!\n\n');

end

F_tilde = -G_v*inv(psi_v);
disp('\tildeF:')
disp(F_tilde)
% disp(inv(psi_v))

%% Extracting the controller dynamics

D_0 = F_tilde(:,1);
E_0 = -F_tilde(:,2:3);
D_1 = F_tilde(:,4);
E_1 = -F_tilde(:,5:6);


%% Simulating the system
%First a bounded energy noise signal is generated

signals_s = randn(nOutputs, N_s); % Normally distributed random numbers (2-by-N matrix)
currentEnergy_s = sum(sum(signals_s.^2)); % Sum of squared values of all elements
scalingFactor_s = sqrt(NoiseEnergy / currentEnergy_s);
noise_s = signals_s * scalingFactor_s;


% simulation here; 

Y_s = zeros(nOutputs,N_s+2); 
U_s = zeros(nInputs,N_s+2); 

Y_s(:,1) = [-0.04;-0.04];
Y_s(:,2) = [-0.00;-0.0];

for kk = 1:N_s_1
    Y_s(:,kk+2) = -P_s1*Y_s(:,kk+1) -P_s0 * Y_s(:,kk) +noise_s(:, kk); %without controller
end


for kk = N_s_1+1:N_s

    U_s(:,kk+2) = -F_tilde(:,1:3)*[U_s(:,kk);Y_s(:,kk)] - F_tilde(:,4:end)*[U_s(:,kk+1);Y_s(:,kk+1)]; %control dynamics
    Y_s(:,kk+2) = -R_s(:,1:3)*[U_s(:,kk);Y_s(:,kk)] - R_s(:,4:end)*[U_s(:,kk+1);Y_s(:,kk+1)]+noise_s(:,kk);%with controller
end

% Create a vector for the column indices
t = 1:(N_s + 2);

% Create a figure window
figure;
% set(gcf, 'PaperUnits', 'inches', 'PaperPosition', [0 0 11 8.5], 'Units', 'inches', 'Position', [0 0 11 8.5]);

% Plot the first row of y in the first subplot
subplot(1, 2, 1);
plot(t, Y_s(1, :), '-','Color', [0, 0.5, 0],'LineWidth', 2);
hold on;
plot(t, Y_s(2, :), '-b','LineWidth', 2);
xline(N_s_1+L, '--k', 'LineWidth', 2, 'color', 'r', 'DisplayName', 'Controller On');
% text(N_s_1+L, max(Y_s(1,:)), 'Controller Activated', 'FontSize', 18, 'Interpreter', 'latex', 'VerticalAlignment', 'middle');
xlabel('$t$', 'FontWeight', 'bold', 'FontSize', 18, 'Interpreter', 'latex');
ylabel('$y(t)\; (rad)$', 'FontWeight', 'bold', 'FontSize', 18, 'Interpreter', 'latex');
set(gca, 'FontSize', 18, 'FontWeight', 'bold');
axis tight;
grid on;
box on;
grid minor;
set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
legend({'$\alpha(t)$', '$\beta(t)$'}, 'Interpreter', 'latex','FontSize', 18, 'FontWeight', 'bold');
legend show;
title('(a)', 'FontSize', 18, 'FontWeight', 'bold', 'Interpreter', 'latex');
hold off;

% Plot the control input in the second subplot
subplot(1, 2, 2);
plot(t, U_s(1, :), '-','Color', [0.6, 0.2, 0.8],'LineWidth', 2);
hold on;
xline(N_s_1+L, '--k', 'LineWidth', 2, 'color', 'r', 'DisplayName', 'Controller On');
% text(N_s_1+L, max(U_s(1,:)), 'Controller Activated', 'FontSize', 18, 'Interpreter', 'latex', 'VerticalAlignment', 'middle');
xlabel('$t$', 'FontWeight', 'bold', 'FontSize', 18, 'Interpreter', 'latex');
ylabel('$\tau(t) \; (N.m)$', 'FontWeight', 'bold', 'FontSize', 18, 'Interpreter', 'latex');
set(gca, 'FontSize', 18, 'FontWeight', 'bold');
axis tight;
grid on;
box on;
grid minor;
set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
title('(b)', 'FontSize', 18, 'FontWeight', 'bold', 'Interpreter', 'latex');
hold off;