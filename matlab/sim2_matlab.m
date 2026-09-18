%% Controllo d'assetto TVC di un razzo modello
% Modello ricavato da simulazione OpenRocket (motore C11-0)

clear; clc; close all;

%% --- Parametri fisici (dal CSV di OpenRocket) ---
T = 11;        % Spinta media durante la combustione [N]
L = 0.255;    % Distanza CG -> punto di gimbal [m]
I = 0.014;    % Momento d'inerzia longitudinale (beccheggio) [kg*m^2]
t_burn = 0.8;  % Durata della fase di spinta [s]

K = T*L/I;    % Guadagno del sistema
fprintf('Guadagno del razzo K = %.1f rad/s^2 per rad di gimbal\n', K);

%% --- Modello del razzo (plant): doppio integratore ---
% theta(s) / delta(s) = K / s^2
s = tf('s');
plant = K / s^2;

%% --- Controllore PID (valori iniziali, poi li tareremo) ---
Kp = 4;
Ki = 0;      % Partiamo senza I: il sistema non ha errore stazionario di gradino
Kd = 0.4;

Tf = 0.01;              
% Costante di tempo del filtro sul derivativo [s] (~100 Hz)
C = pid(Kp, Ki, Kd, Tf);

%% --- Sistema ad anello chiuso ---
sys_cl = feedback(C*plant, 1);

%% --- Grafico 1: risposta al gradino ---
% Chiediamo al razzo: "portati a 0 rad partendo da un errore iniziale"
figure('Name','Risposta al gradino ad anello chiuso');
step(sys_cl, 0:0.001:t_burn);
grid on;
title('Assetto del razzo — risposta al gradino');
ylabel('Angolo di beccheggio \theta [rad]');
xlabel('Tempo [s]');

%% --- Grafico 2: margini di stabilità (Bode) ---
% "validate system stability" letterale dell'annuncio di lavoro
figure('Name','Analisi di stabilità');
margin(C*plant);
grid on;

%% --- Grafico 3: comando del gimbal nel tempo ---
% Dobbiamo controllare che il motore non venga chiesto di ruotare
% oltre il limite fisico (tipicamente +-5 deg)
t = 0:0.001:t_burn;
[y, t_out] = step(sys_cl, t);            % uscita: angolo assetto
u = lsim(C/(1+C*plant), ones(size(t)), t); % ingresso: comando gimbal

figure('Name','Comando di gimbal');
plot(t_out, rad2deg(u), 'LineWidth', 1.5);
grid on;
yline(5, '--r', 'Limite +5°');
yline(-5,'--r', 'Limite -5°');
title('Comando di gimbal \delta(t)');
xlabel('Tempo [s]'); ylabel('\delta [deg]');

%% --- Stampa dei margini in console ---
[Gm, Pm, ~, ~] = margin(C*plant);
fprintf('\n--- Prestazioni ---\n');
fprintf('Margine di guadagno: %.1f dB\n', 20*log10(Gm));
fprintf('Margine di fase:      %.1f deg\n', Pm);

info = stepinfo(sys_cl);
fprintf('Tempo di salita:      %.3f s\n', info.RiseTime);
fprintf('Tempo di assestamento:%.3f s\n', info.SettlingTime);
fprintf('Sovraelongazione:     %.1f %%\n', info.Overshoot);