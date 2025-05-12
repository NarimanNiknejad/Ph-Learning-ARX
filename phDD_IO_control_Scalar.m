%% Cleaning

clc;
clear;
close all;

%% Initializing the Parameters

N = 1;  %number of collected data
N_s = 100; % number of steps for simulation
N_s_1 = 15; %number of steps of simulation without stabilization
N_s_2 = N_s - N_s_1; %number of steps of simulation with stabilization
T = 2; % burn-in period for data collection

L = 1;  %degree of the system

nOutputs = 1;    %dimension of the output vector
nInputs = 1;    %dimension of the input vector

q = nOutputs + nInputs;

epsilon = 0.12; %uncertain physics information

NoiseEnergy = 0.01; % based on the characteristics of the system 


%% Nominal system parameters

Q_n0 = 0.3;
P_n0 = -1.021;

R_n = [-Q_n0, P_n0];



%% Actual System Parameters

Q_s0 = 0.4;
P_s0 = -1.035;

R_s = [-Q_s0, P_s0];

epsilon_s = norm(R_s-R_n,2);

disp(epsilon_s); % Take epsilon larger than this value by some buffer

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

% data collection here; 

Y = zeros(nOutputs,N+T+2); 
Y(:,1) = [0.5]';

U = -1 +2*rand(nInputs,N+T+1);

for kk = 1:N+T
    Y(:,kk+1) = -R_s(:,1:2)*[U(:,kk);Y(:,kk)] +noise(:,kk);
end

% removing the burn-in values
Y_1 = Y(:,T+1:end-1);
U_1 = U(:,T+1:end-1);

[H_1, H_2] = modifiedHankel(U_1, Y_1, L); % forming the modified Hankel

%% Forming the data-conformity set

O_n = [NoiseEnergy*eye(nOutputs), zeros(nOutputs,N-L+1);
        zeros(nOutputs,N-L+1)', -eye(N-L+1)];

O_data = [eye(nOutputs),H_2;zeros(q*L,nOutputs),H_1]*O_n*[eye(nOutputs),H_2;zeros(q*L,nOutputs),H_1]'; %dimension is qL + p by qL + p

O_data_projected = [ [zeros(nOutputs, q*(L-1)), zeros(nOutputs,nInputs),-eye(nOutputs)], zeros(nOutputs,q*L); zeros(q*L,q*L), eye(q*L) ]'*O_data*[ [zeros(nOutputs, q*(L-1)), zeros(nOutputs,nInputs),-eye(nOutputs)], zeros(nOutputs,q*L); zeros(q*L,q*L), eye(q*L) ]; %dimension is 2qL by 2qL

%% optimization problem (finding stabilizng controller) both phys and data 


psi = sdpvar(q*L,q*L,'symmetric');
G = sdpvar(nInputs, q*L);
I_tilde = [zeros(q*(L-1),q), eye(q*(L-1))];

tau_dt = sdpvar(1,1);
% tau_ph = sdpvar(1,1);
e = sdpvar(1,1);

miu_dt =sdpvar(1,1);
miu_ph =sdpvar(1,1);

constraint_1 =  miu_dt * O_data_projected + miu_ph * O_phys_projected>= 0.0;
constraint_2 = [psi, -[I_tilde*psi;G;zeros(nOutputs,q*L)],[I_tilde*psi;G;zeros(nOutputs,q*L)];
                -[I_tilde*psi;G;zeros(nOutputs,q*L)]', -psi,zeros(q*L);
                [I_tilde*psi;G;zeros(nOutputs,q*L)]',zeros(q*L),psi]  - (1-tau_dt)*[O_phys_projected,zeros(2*q*L,q*L);
                                                                zeros(q*L,2*q*L),zeros(q*L,q*L)] -tau_dt*[O_data_projected,zeros(2*q*L,q*L);
                                                                                                  zeros(q*L,2*q*L),zeros(q*L,q*L)] >=e;

constraint_3 = tau_dt>=0;
constraint_4 = tau_dt<=1;
constraint_5 = e>=0.0001;




constraints =   constraint_1+constraint_2 + constraint_3 +constraint_4 + constraint_5;
cost = e;


ops = sdpsettings('solver','mosek','verbose',0);
diagnostics = optimize(constraints, cost, ops);

if(diagnostics.problem == 0)

    psi_v = value(psi);
    G_v = value(G);
    % tau_ph_v = value(tau_ph);
    tau_dt_v = value(tau_dt);


else

    fprintf('Error! Solution is infeasible!\n\n');

end

F_tilde = -G_v*inv(psi_v);
disp(F_tilde)
disp(inv(psi_v))

o_new = (1- tau_dt_v)*O_phys + tau_dt_v*O_data;


%% Simulating the system
%First a bounded energy noise signal is generated

signals_s = randn(nOutputs, N_s); % Normally distributed random numbers (2-by-N matrix)
currentEnergy_s = sum(sum(signals_s.^2)); % Sum of squared values of all elements
scalingFactor_s = sqrt(NoiseEnergy / currentEnergy_s);
noise_s = signals_s * scalingFactor_s;


% simulation here; 

Y_s = zeros(nOutputs,N_s+1); 
U_s = zeros(nInputs,N_s+1); 

Y_s(:,1) = [0.6];

for kk = 1:N_s_1
    Y_s(:,kk+1) = -P_s0 * Y_s(:,kk) +noise_s(:, kk); %without controller
end


for kk = N_s_1+1:N_s

    Y_s(:,kk+1) = -R_s(:,1:end)*[U_s(:,kk);Y_s(:,kk)] + noise_s(:, kk);%with controller
    U_s(:,kk+1) = -F_tilde(:,1)*U_s(:,kk)-F_tilde(:,2)*Y_s(:,kk); %control dynamics
end


%% plotting the results

% Create a vector for the column indices
t = 1:(N_s+1);

% Plot the first row of y
figure;
plot(t, Y_s(1, :), '-', 'DisplayName', '$y_1$', 'LineWidth', 2.5, 'MarkerSize', 6);

hold on;

% Add a vertical line at t = N_s_1+L to indicate the start of the controller
xline(N_s_1+L, '--k', 'LineWidth', 2,'color','r', 'DisplayName', 'Controller On');

% Add text annotation
% text(N_s_1+L, max(Y_s(1,:)), 'Controller Activated','Rotation', 90, 'FontSize', 18, 'Interpreter', 'latex', 'VerticalAlignment', 'middle');

% Add labels and legend
xlabel('$t$', 'FontWeight', 'bold', 'FontSize', 18, 'Interpreter', 'latex');
ylabel('$y(t)$', 'FontWeight', 'bold', 'FontSize', 18, 'Interpreter', 'latex');

% Customize axis properties
set(gca, 'FontSize', 18, 'FontWeight', 'bold');
axis tight;
grid on;
box on;

% Enhance grid lines
grid minor;
set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);

hold off;


%% Draw the information sets

% Generate a grid of points in 2D space
[x, y] = meshgrid(linspace(-0.5, -0.1, 100), linspace(-1.2, -0.85, 100));
points = [x(:)'; y(:)']; % 2xN matrix of points

% Evaluate the quadratic form for each point
values = zeros(1, size(points, 2));
values_1 = zeros(1, size(points, 2));
values_2 = zeros(1, size(points, 2));

for i = 1:size(points, 2)
    R = [1; points(:, i)]; % [I; R^T]
    values(i) = R' * O_data * R;
    values_1(i) = R' * O_phys * R;
    values_2(i) = R' * o_new * R;
end

% Reshape the results to match the grid
values = reshape(values, size(x));
values_1 = reshape(values_1, size(x));
values_2 = reshape(values_2, size(x));

% Plot the ellipsoid
% figure;

% Create axes
axes1 = axes('Parent',figure);
hold(axes1,'on');
hold on;

% Filled contours with distinct colors
contourf(x, y, values_1, [0 0], 'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
contourf(x, y, values, [0 0], 'FaceColor', 'red', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
contourf(x, y, values_2, [0 0], 'FaceColor', 'green', 'FaceAlpha', 0.3, 'EdgeColor', 'none');


% Outline contours
contour(x, y, values, [0 0], 'r', 'LineWidth', 1.5);
contour(x, y, values_1, [0 0], 'b', 'LineWidth', 1.5);

% Specify a point
point = R_s; % Example point coordinates
plot(point(1), point(2), 'ko', 'MarkerFaceColor', 'green', 'MarkerSize', 8);

% Create text
text('Parent',axes1,'FontWeight','bold','FontSize',22,'Interpreter','latex',...
    'String','$\tilde{R}^\star$',...
    'Position',[-0.386934306569343 -1.02058394160584 -8.88178419700125e-16],...
    'Color',[0 1 0]);

% Create ylabel
ylabel('$P_0$ ','FontWeight','bold','FontSize',18,'FontName','Avenir',...
    'Interpreter','latex');

% Create xlabel
xlabel('$-Q_0$ ','FontWeight','bold','FontSize',18,'Interpreter','latex');

% 
% % Create text
% text('Parent',axes1,'FontWeight','bold','FontSize',22,'Interpreter','latex',...
%     'String','$\mathcal{E}_{data}$',...
%     'Position',[-0.166131386861314 -0.957080291970803 -8.88178419700125e-16],...
%     'Color',[1 0 0]);
% 
% % Create text
% text('Parent',axes1,'FontWeight','bold','FontSize',22,'Interpreter','latex',...
%     'String','$\mathcal{E}^0_{pk}$',...
%     'Position',[-0.416496350364964 -0.919270072992701 -8.88178419700125e-16],...
%     'Color',[0 0 1]);
% 
% % Create text
% text('Parent',axes1,'FontWeight','bold','FontSize',22,'Interpreter','latex',...
%     'String','$\mathcal{E}^1_{pk}$',...
%     'Position',[-0.416496350364964 -0.919270072992701 -8.88178419700125e-16],...
%     'Color','green');

% Adjusting axis for better visualization
axis equal;
set(gca, 'FontSize', 18, 'FontWeight', 'bold');
grid on;
box on;

hold off;

