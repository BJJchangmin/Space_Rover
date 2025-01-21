close all;

filename{1} = 'data_FL.csv';
filename{2} = 'data_FR.csv';
filename{3} = 'data_RL.csv';
filename{4} = 'data_RR.csv';
filename{5} = 'data_trunk.csv';

%Leg Data Load 
for i = 1:1:4 
    Arr_Leg{i} = table2array(readtable(filename{i}));
end

%Trunk Data Load
Arr_trunk = table2array(readtable(filename{5}));


t = Arr_Leg{1}(:,1);

for i = 1:1:4

    sus_pos_ref{i} = Arr_Leg{i}(:,2);
    sus_pos{i} = Arr_Leg{i}(:,3);
    sus_vel{i} = Arr_Leg{i}(:,4);
    sus_torque{i} = Arr_Leg{i}(:,5);

    steer_pos_ref{i} = Arr_Leg{i}(:,6);
    steer_pos{i} = Arr_Leg{i}(:,7);
    steer_vel{i} = Arr_Leg{i}(:,8);
    steer_torque{i} = Arr_Leg{i}(:,9);

    drive_vel_ref{i} = Arr_Leg{i}(:,10);
    drive_pos{i} = Arr_Leg{i}(:,11);
    drive_vel{i} = Arr_Leg{i}(:,12);
    drive_torque{i} = Arr_Leg{i}(:,13);


end


Ts = t(2,1)-t(1,1);
for i = 1:length(sus_pos_ref)
    t(i,1) = (i-1)*Ts;
end

%%%%%%%%%%%%%%%%%%%%% ID Parameter %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_repeat = 1;
n_transient = 0;

Fs = 1000; Ts = 1/Fs; %% sampling frequency,time
end_time = 10;
N_time = end_time/Ts; %Number of Data one cycle


start_idx =  3*1000 + n_transient*N_time;
end_idx = start_idx + (n_repeat-n_transient)*N_time;
% 
% input_data = sus_torque{1}(start_idx:end_idx);
% output_data = sus_vel{1}(start_idx:end_idx);

input_data = steer_torque{1}(start_idx:end_idx);
output_data = steer_vel{1}(start_idx:end_idx);

N= length(input_data);
if length(output_data) ~= N
    error('torque and vel length are different')
end





U = fft(input_data);
Y = fft(output_data);


% 오토 스펙트럼(입력) 및 크로스 스펙트럼(출력-입력)
puu = U .* conj(U);  % P_uu
puy = Y .* conj(U);  % P_uy
% 전달함수 추정: H(omega) = P_uy / P_uu
H = puy ./ puu;

%% 3) 크기/위상 (전체 스펙트럼)
magFull   = 20 * log10(abs(H));
phaseFull = (180 / pi) * angle(H);

%% 4) 주파수 축 생성

% 표준 FFT 결과로는 0 ~ Fs*(N-1)/N 까지 N개 샘플
fAxisFull = (0:N-1)' * (Fs / N);

% 보통 양쪽 대칭이라 0 ~ Fs/2까지만 사용
halfN  = floor(N / 2) + 1;
fAxis  = fAxisFull(1:halfN);
magTra = magFull(1:halfN);
phaseTra = phaseFull(1:halfN);

%=================== Nominal Model Tunning ===========================%
% J,B Parameter Setting
J = 35;   % 관성 [kg*m^2]
B = 0.1;    % 감쇠(마찰) [N*m*s/rad]

% s = jw 라플라스 변수
s = tf('s');

% 명목 모델: G(s) = 1/(J s + B)
G_nominal = 1/(J*s + B);

%=================== Nominal Bode Plot 계산 ========================%
% Bode 함수를 fAxis(Hz)에 맞춰서 계산해야 하므로,
% 각 주파수 fAxis(Hz)에 대응하는 각진동수 w = 2*pi*fAxis
wVec = 2*pi*fAxis;  % rad/s

[magNom, phaseNom] = bode(G_nominal, wVec);
% bode() 호출 결과는 3차원 배열이므로 squeeze()로 1D로 만듦
magNom    = squeeze(magNom);     % 전송함수 크기 (절댓값)
phaseNom  = squeeze(phaseNom);   % 전송함수 위상 (deg)

% magNom은 절댓값이므로, dB 스케일 변환
magNom_dB = 20 * log10(magNom);

%=================== 결과 비교 Plot ================================%
figure('Name','Torque->Velocity : FFT 식별 vs 명목 모델','Color','w');

% (1) Magnitude
subplot(2,1,1)
semilogx(fAxis, magTra, 'b-', 'LineWidth', 1.5); hold on;
semilogx(fAxis, magNom_dB, 'r--', 'LineWidth', 1.5);
grid on;
xlim([0.1 100])
xlabel('Frequency [Hz]'); 
ylabel('Magnitude [dB]');
legend('Identified (FFT)', 'Nominal Model');
title('Torque->Velocity : Magnitude');

% (2) Phase
subplot(2,1,2)
semilogx(fAxis, phaseTra, 'b-', 'LineWidth', 1.5); hold on;
semilogx(fAxis, phaseNom,  'r--', 'LineWidth', 1.5);
grid on;
xlabel('Frequency [Hz]'); 
xlim([0.1 100])
ylabel('Phase [deg]');
legend('Identified (FFT)', 'Nominal Model');
title('Torque->Velocity : Phase');