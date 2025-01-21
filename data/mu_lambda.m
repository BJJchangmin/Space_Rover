w_min = -1;         % 최소 속도
w_max = 1;          % 최대 속도
w = linspace(w_min, w_max, 100); % 속도 배열 생성
%% KNEE
%%% Motor vel 2 %%%% FL
e = 2.71828; T_brk = 0.8;
% Breakaway Friction = Coulomb + Static friction
T_c = 0.5; %Coulomb Friction
w_brk = 0.4; %Breakaway Friction Velocity : Peak Velocity for Stribeck Friction
w_st = sqrt(2)*w_brk; % Stribeck velocity threshold
w_coul = w_brk/10; % Coulomb velocity threshold
f = -0.01; % Viscous Friction Coefficient, 일종의 기울기
bias = 0;
Fric = sqrt(2)*e*(T_brk - T_c) .* exp(-(w ./ w_st).^2) .* (w ./ w_st) + ...
    T_c .* tanh(w ./ w_coul) + f .* w + bias;
figure(2)
plot(w, Fric/1.2, 'LineWidth', 2);
hold on;
xlabel('Slip Ratio', 'FontSize', 12);
ylabel('Mu', 'FontSize', 12);
title('Mu-Lamda Function', 'FontSize', 14);
grid on;
% 
% Fric = sqrt(2)*e*(T_brk - T_c) .* exp(-(0.2 ./ w_st).^2) .* (0.2 ./ w_st) + ...
%     T_c .* tanh(0.2 ./ w_coul) + f .* 0.2 + bias;