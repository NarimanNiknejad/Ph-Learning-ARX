clc;
clear;
close all;

%% Discrete-time model alpha_2

syms alpha_2 beta_2 alpha_1 beta_1 alpha beta tau alpha_dd
syms J_r m_p L_r L_p C_r g del

% Define the standard rules
% alpha_dd = (alpha_2 - 2*alpha_1 + alpha) / (del^2);
alpha_d = (alpha_1 - alpha) / del;
beta_dd = (beta_2 - 2*beta_1 + beta) / (del^2);
beta_d = (beta_1 - beta) / del;

% Equations to solve
eq1 = (J_r + m_p * L_r^2) * alpha_dd - (m_p * L_p * L_r) * beta_dd + C_r * alpha_d == tau;
eq2 = -(m_p * L_p * L_r) * alpha_dd +  (m_p * L_p^2) * beta_dd - m_p * L_p * g * beta == 0;


% Solve for beta_dd
sol_alpha_dd = simplify(solve(eq2, alpha_dd));

eq_1_inc = (J_r + m_p * L_r^2) * sol_alpha_dd - (m_p * L_p * L_r) * beta_dd + C_r * alpha_d == tau;
pretty(simplify(eq_1_inc))

beta_2_sol = simplify(solve(eq_1_inc, beta_2));
pretty(beta_2_sol)

beta_2_collected = (collect(beta_2_sol, [alpha_1, beta_1, alpha, beta, tau]));
latex(-beta_2_collected)
pretty(-beta_2_collected)