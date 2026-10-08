%% AGROVER MR 25.26 — Simulation cinématique skid-steer
%  Cas : Avancer | Reculer | Tourner | Pivot
%  Modèle : v = (vR+vL)/2  |  omega = (vR-vL)/L

clear; clc; close all;

%% Paramètres robot
L  = 0.300;  % Voie [m]
dt = 0.05;   % Pas de temps [s]
T  = 8;      % Durée par cas [s]
t  = 0:dt:T;
N  = length(t);

%% Définition des 6 cas
% [vL, vR, nom, couleur]
cas(1).vL  =  0.20;  cas(1).vR  =  0.20;
cas(1).nom = 'Avancer';           cas(1).col = [0.18 0.45 0.70];

cas(2).vL  = -0.20;  cas(2).vR  = -0.20;
cas(2).nom = 'Reculer';           cas(2).col = [0.85 0.33 0.10];

cas(3).vL  =  0.05;  cas(3).vR  =  0.20;
cas(3).nom = 'Tourner gauche';    cas(3).col = [0.18 0.63 0.27];

cas(4).vL  =  0.20;  cas(4).vR  =  0.05;
cas(4).nom = 'Tourner droite';    cas(4).col = [0.93 0.69 0.13];

cas(5).vL  = -0.15;  cas(5).vR  =  0.15;
cas(5).nom = 'Pivot gauche';      cas(5).col = [0.55 0.18 0.60];

cas(6).vL  =  0.15;  cas(6).vR  = -0.15;
cas(6).nom = 'Pivot droit';       cas(6).col = [0.75 0.22 0.17];

%% Simulation
for s = 1:6
    vL = cas(s).vL * ones(1,N);
    vR = cas(s).vR * ones(1,N);
    x = zeros(1,N); y = zeros(1,N); th = zeros(1,N);
    v = zeros(1,N); w = zeros(1,N);
    for i = 2:N
        v(i) = (vR(i-1) + vL(i-1)) / 2;
        w(i) = (vR(i-1) - vL(i-1)) / L;
        th(i) = th(i-1) + w(i)*dt;
        x(i)  = x(i-1) + v(i)*cos(th(i-1) + w(i)*dt/2)*dt;
        y(i)  = y(i-1) + v(i)*sin(th(i-1) + w(i)*dt/2)*dt;
    end
    cas(s).x = x; cas(s).y = y; cas(s).th = th;
    cas(s).v = v; cas(s).w = w;
    cas(s).v_val = cas(s).vR/2 + cas(s).vL/2;
    cas(s).w_val = (cas(s).vR - cas(s).vL) / L;
end

%% Figures — une par cas
for s = 1:6
    sc = cas(s);

    figure('Name', ['AGROVER — ' sc.nom], 'Color','w', ...
           'Position', [80 + s*30, 80 + s*20, 1000, 380]);

    sgtitle(['\fontsize{13}\bf AGROVER MR 25.26 — ' sc.nom], ...
            'FontName','Arial');

    %% ── Trajectoire ──────────────────────────────────────────
    subplot(1, 3, 1);
    hold on; grid on; axis equal;

    plot(sc.x, sc.y, '-', 'Color', sc.col, 'LineWidth', 2.5);
    plot(sc.x(1), sc.y(1), 'o', 'MarkerSize', 10, ...
         'MarkerFaceColor', sc.col, 'MarkerEdgeColor','w','LineWidth',1.5);
    plot(sc.x(end), sc.y(end), 's', 'MarkerSize', 10, ...
         'MarkerFaceColor','w','MarkerEdgeColor',sc.col,'LineWidth',2);

    % Flèches de direction toutes les 2 s
    al = max([max(sc.x)-min(sc.x), max(sc.y)-min(sc.y), 0.05]) * 0.14;
    for i = 1:round(2/dt):N
        quiver(sc.x(i), sc.y(i), al*cos(sc.th(i)), al*sin(sc.th(i)), ...
               0, 'Color', sc.col*0.65, 'LineWidth',1.2,'MaxHeadSize',2);
    end

    xlabel('x [m]'); ylabel('y [m]');
    title('\bf Trajectoire', 'FontSize', 11);

    % Encadré valeurs
    annotation_str = sprintf('v = %.2f m/s\n\\omega = %.2f rad/s', sc.v_val, sc.w_val);
    text(0.04, 0.96, annotation_str, ...
         'Units','normalized','FontSize',10,'FontWeight','bold', ...
         'Color', sc.col, 'VerticalAlignment','top', ...
         'BackgroundColor','w','EdgeColor',[0.8 0.8 0.8],'Margin',4);

    %% ── Vitesse linéaire v(t) ────────────────────────────────
    subplot(1, 3, 2);
    hold on; grid on;
    area(t, sc.v, 'FaceColor', sc.col, 'FaceAlpha', 0.15, 'EdgeColor','none');
    plot(t, sc.v, '-', 'Color', sc.col, 'LineWidth', 2.5);
    yline(0,        'k--', 'LineWidth', 0.8);
    yline(sc.v_val, '--', sprintf('%.2f m/s', sc.v_val), ...
          'Color',[0.35 0.35 0.35],'LineWidth',1.2,'FontSize',9, ...
          'LabelHorizontalAlignment','right');
    ylim([-0.30 0.30]);
    xlabel('t [s]'); ylabel('v [m/s]');
    title('\bf Vitesse linéaire  v', 'FontSize', 11);

    %% ── Vitesse angulaire ω(t) ───────────────────────────────
    subplot(1, 3, 3);
    hold on; grid on;
    area(t, sc.w, 'FaceColor', sc.col*0.75, 'FaceAlpha', 0.15, 'EdgeColor','none');
    plot(t, sc.w, '-', 'Color', sc.col*0.75, 'LineWidth', 2.5);
    yline(0,        'k--', 'LineWidth', 0.8);
    yline(sc.w_val, '--', sprintf('%.2f rad/s', sc.w_val), ...
          'Color',[0.35 0.35 0.35],'LineWidth',1.2,'FontSize',9, ...
          'LabelHorizontalAlignment','right');
    ylim([-1.5 1.5]);
    xlabel('t [s]'); ylabel('\omega [rad/s]');
    title('\bf Vitesse angulaire  \omega', 'FontSize', 11);
end
