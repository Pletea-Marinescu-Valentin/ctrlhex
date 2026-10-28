function T = restore_simulation(vehicle, saveFigures)
% Hexacopter X with motor 1 stopped at 2 s and released at 7 s: PX4 one-step restore versus the gradual
% restore (CA_RESTORE_SPIN, CA_RESTORE_TIME). Rigid body with 6 DOF, motors with dead time and time
% constant, PX4 attitude/rate cascade, PX4 pseudo-inverse allocation with sequential desaturation,
% opposite rotor reversible (CA_FAILURE_MODE 2).
%   T = restore_simulation('demonstrator')
%   T = restore_simulation('small')
% Measured on the vehicles: hover command and rate gains. Mass, arm, inertia and motor times are estimates.

if nargin < 1, vehicle = 'demonstrator'; end
if nargin < 2, saveFigures = true; end

switch vehicle
    case 'demonstrator'                 % 3115 900KV, 10" propellers, 6S
        D.m = 2.50; D.L = 0.30; D.J = diag([0.030 0.030 0.055]); D.kq = 0.020;
        D.tauM = 0.060; D.tStart = 0.15; D.hover = 0.21;
        C.rateK = [0.60 0.65 1.0]; C.rateI = [0.30 0.30 0.15]; C.intLim = [0.4 0.4 0.3];
        figTitle = 'Large hexacopter simulation';
    case 'small'                        % KOKO 1505 3450KV, 4" propellers, 3S
        D.m = 0.70; D.L = 0.12; D.J = diag([0.0035 0.0035 0.0060]); D.kq = 0.012;
        D.tauM = 0.035; D.tStart = 0.12; D.hover = 0.31;
        C.rateK = [0.35 0.40 1.0]; C.rateI = [0.20 0.20 0.10]; C.intLim = [0.3 0.3 0.3];
        figTitle = 'Small hexacopter simulation';
    otherwise
        error('vehicle must be ''demonstrator'' or ''small''');
end
D.rev   = 0.4;                          % CA_REV_THR_FRAC
D.idle  = 0.02;                         % DSHOT_MIN
D.Tmax  = D.m * 9.81 / (6 * D.hover);
% asymmetries of a real vehicle; without them the model is symmetric and the restore kick vanishes
D.cg    = [0.004; -0.006; 0];
D.dT    = [0.03 -0.02 0.01 -0.03 0.02 -0.01]';
D.noise = deg2rad(0.3);

C.attP  = [6.5 6.5 2.8];
C.rateP = [0.15 0.15 0.20];
C.rateD = [0.003 0.003 0];
C.rateMax = deg2rad([220 220 200]);
C.zP = 1.0; C.zVelP = 4.0; C.zVelI = 2.0;

S.h0 = 1.0; S.tStop = 2.0; S.tRelease = 7.0; S.tEnd = 11.0;
S.spin = 0.3; S.blend = 1.0;

root = fileparts(fileparts(mfilename('fullpath')));
outDir = fullfile(root, 'results', 'figures', vehicle);
if saveFigures && ~exist(outDir, 'dir'), mkdir(outDir); end

Rstep = simulate(D, C, S, false);
Rgrad = simulate(D, C, S, true);

%% Figure 1: thrust sharing with motor 1 stopped
nominal = mean(Rgrad.u(Rgrad.t > 1.0 & Rgrad.t < 1.9, :), 1, 'omitnan');
failed  = mean(Rgrad.u(Rgrad.t > 4.0 & Rgrad.t < 6.9, :), 1, 'omitnan'); failed(1) = 0;
ideal = [ones(1, 6); 0 0 1.5 1.5 1.5 1.5] * D.hover;

f = new_figure(17, 9);
tl = tiledlayout(f, 1, 1, 'Padding', 'compact');
ax = nexttile(tl); hold(ax, 'on');
b = bar(ax, 1:6, [nominal; failed]', 0.8, 'EdgeColor', 'none');
b(1).FaceColor = palette().blue; b(2).FaceColor = palette().orange;
xp = b(2).XEndPoints;
ha = plot(ax, [b(1).XEndPoints xp], [ideal(1, :) ideal(2, :)], 'd', 'MarkerSize', 6, ...
          'MarkerFaceColor', palette().ink, 'MarkerEdgeColor', 'w', 'LineStyle', 'none');
yline(ax, 1, '--', 'motor limit', 'Color', palette().muted, 'FontSize', 8, 'LabelColor', palette().ink2, ...
      'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
text(ax, xp(1), 0.04, 'stopped', 'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', palette().ink2, 'FontName', plot_font());
text(ax, xp(2), max(failed(2), 0) + 0.04, 'opposite ≈ 0', 'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', palette().ink2, 'FontName', plot_font());
set(ax, 'XTick', 1:6, 'XTickLabel', {'M1 right', 'M2 left', 'M3 front-left', 'M4 rear-right', 'M5 front-right', 'M6 rear-left'});
ylim(ax, [0 1.1]); ylabel(ax, 'Mean hover command [0..1]');
set_title(tl, sprintf('Without motor 1, its opposite carries ≈ 0 and the other four %.2f instead of %.2f (×%.2f)', ...
          mean(failed(3:6)), mean(nominal), mean(failed(3:6)) / mean(nominal)));
add_legend(ax, {'6 motors (simulation)', 'motor 1 stopped (simulation)', 'ideal calculation, symmetric vehicle'}, ...
           [b(1) b(2) ha], 'tile-north');
style_axes(ax);
save_figure(f, outDir, 'sim_thrust_sharing', saveFigures);

%% Figure 2: stop and restore
f = new_figure(17, 15);
tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = nexttile(tl); hold(ax1, 'on'); bands(ax1, S);
h1 = plot(ax1, Rstep.t, Rstep.eul(:, 1), 'Color', palette().orange, 'LineWidth', 1.5);
h2 = plot(ax1, Rgrad.t, Rgrad.eul(:, 1), 'Color', palette().blue, 'LineWidth', 1.5);
ylabel(ax1, 'Roll [°]');
add_legend(ax1, {'one-step restore (PX4 upstream)', 'gradual restore (CTRLHEX)'}, [h1 h2], 'tile-north');
ax2 = nexttile(tl); hold(ax2, 'on'); bands(ax2, S);
plot(ax2, Rstep.t, Rstep.v(:, 2), 'Color', palette().orange, 'LineWidth', 1.5);
plot(ax2, Rgrad.t, Rgrad.v(:, 2), 'Color', palette().blue, 'LineWidth', 1.5);
ylabel(ax2, 'Lateral speed [m/s]');
ax3 = nexttile(tl); hold(ax3, 'on'); bands(ax3, S);
hg = plot(ax3, Rgrad.t, Rgrad.u(:, 3:6), 'Color', palette().muted, 'LineWidth', 0.9);
ho = plot(ax3, Rgrad.t, Rgrad.u(:, 2), 'Color', palette().orange, 'LineWidth', 1.5);
hm = plot(ax3, Rgrad.t, Rgrad.u(:, 1), 'Color', palette().blue, 'LineWidth', 1.5);
yline(ax3, 0, 'Color', palette().base, 'LineWidth', 0.75);
ylabel(ax3, 'Command (CTRLHEX)'); ylim(ax3, [-0.25 0.65]);
add_legend(ax3, {'M1 (stopped, then restored)', 'M2 opposite (reversible)', 'other 4'}, [hm ho hg(1)], 'tile-south');
set_title(tl, sprintf('%s: motor 1 stopped at 2 s, restored at 7 s', figTitle));
xlabel(tl, 'Time [s]', 'Color', palette().ink2, 'FontName', plot_font(), 'FontSize', 9);
for ax = [ax1 ax2 ax3], style_axes(ax); xlim(ax, [1 S.tEnd]); end
linkaxes([ax1 ax2 ax3], 'x');
save_figure(f, outDir, 'sim_stop_and_restore', saveFigures);

%% Numbers
win = @(R, a, b) R.t >= a & R.t <= b;
pk = @(R, a, b, x) max(abs(x(win(R, a, b), :)), [], 'all');
T = table([pk(Rstep, 7, 10, Rstep.eul(:, 1)); pk(Rgrad, 7, 10, Rgrad.eul(:, 1))], ...
          [pk(Rstep, 7, 10, Rstep.w(:, 1)); pk(Rgrad, 7, 10, Rgrad.w(:, 1))], ...
          [pk(Rstep, 7, 10, Rstep.v(:, 2)); pk(Rgrad, 7, 10, Rgrad.v(:, 2))], ...
          'VariableNames', {'max_roll_deg', 'max_roll_rate_dps', 'max_lateral_speed_mps'}, ...
          'RowNames', {'one_step', 'gradual'});
fprintf('\n%s: after the stop (2-4 s) max roll %.1f deg, height lost %.0f cm\n', vehicle, ...
        pk(Rgrad, 2, 4, Rgrad.eul(:, 1)), 100 * (S.h0 - min(Rgrad.h(win(Rgrad, 2, 4)))));
fprintf('after the restore (7-10 s):\n');
disp(T);
end

function R = simulate(D, C, S, gradual)
% 6 s of settling before t = 0 so the integrators absorb the asymmetries; recording starts at t = 0
dt = 1e-3; tSettle = 6; N = round((S.tEnd + tSettle) / dt); g = 9.81;

% airframe 6001: normalised [x forward, y right] positions and spin direction
pos  = [0 0.5; 0 -0.5; 0.43 -0.25; -0.43 0.25; 0.43 0.25; -0.43 -0.25];
spin = [-1 1 -1 1 1 -1];
rPhys = pos / 0.5 * D.L;
opposite = [2 1 4 3 6 5];

% PX4 effectiveness (roll, pitch, yaw, thrust) with CA_ROTOR*_CT = 6.5, KM = 0.05; scaling computed once
B = 6.5 * [-pos(:, 2)'; pos(:, 1)'; 0.05 * spin; ones(1, 6)];
scale = px4_scale(pinv(B));
M = pinv(B) ./ scale;

p = [0; 0; -S.h0]; v = zeros(3, 1); q = [1; 0; 0; 0]; w = zeros(3, 1);
uAct = D.hover * ones(6, 1); tStart = NaN(6, 1);
rateInt = zeros(3, 1); zInt = 0; wF = w; wFPrev = w;
rng(7);
aGyro = 2 * pi * 40 * dt / (1 + 2 * pi * 40 * dt);       % IMU_GYRO_CUTOFF 40 Hz
active = true(6, 1); reversible = false(6, 1);
sRest = 1; restoring = false; tLastUpdate = -1;

nLog = floor(N / 5); R.t = zeros(nLog, 1); R.eul = zeros(nLog, 3); R.w = zeros(nLog, 3);
R.v = zeros(nLog, 3); R.h = zeros(nLog, 1); R.u = zeros(nLog, 6); k = 0;

for n = 1:N
    t = (n - 1) * dt - tSettle;
    if abs(t) < dt / 2, v(1:2) = 0; p(1:2) = 0; end

    if abs(t - S.tStop) < dt / 2
        active(1) = false; reversible(opposite(1)) = true; restoring = false;
        M = pinv(B .* [0 1 1 1 1 1]) ./ scale;
    end
    if abs(t - S.tRelease) < dt / 2
        active(1) = true;
        if gradual
            restoring = true; sRest = 0; tLastUpdate = -1;
            M = pinv(B .* [0 1 1 1 1 1]) ./ scale;
        else
            reversible(:) = false; M = pinv(B) ./ scale;
        end
    end
    if restoring
        x = min(max((t - S.tRelease - S.spin) / S.blend, 0), 1);
        s = x^2 * (3 - 2 * x);
        if x >= 1
            restoring = false; sRest = 1; reversible(:) = false; M = pinv(B) ./ scale;
        elseif s > sRest && t - tLastUpdate >= 0.01 - 1e-9
            sRest = s; tLastUpdate = t;
            M = pinv(B .* [sRest 1 1 1 1 1]) ./ scale;
        end
    end

    Rb = rotation(q);
    vzSp = max(min(-C.zP * (S.h0 - (-p(3))), 1), -1);
    eV = vzSp - v(3);
    zInt = zInt + C.zVelI * eV * dt;
    aSp = C.zVelP * eV + zInt;
    T = D.hover * (1 - aSp / g) / max(Rb(3, 3), 0.5);

    qe = q_mul(q_conj(q), [1; 0; 0; 0]);
    eq = 2 * sign(qe(1) + (qe(1) == 0)) * qe(2:4);
    rateSp = max(min(C.attP(:) .* eq, C.rateMax(:)), -C.rateMax(:));

    wF = wF + aGyro * (w + D.noise * randn(3, 1) - wF);
    wDot = (wF - wFPrev) / dt; wFPrev = wF;
    eR = rateSp - wF;
    rateInt = max(min(rateInt + C.rateK(:) .* C.rateI(:) .* eR * dt, C.intLim(:)), -C.intLim(:));
    tau = C.rateK(:) .* (C.rateP(:) .* eR - C.rateD(:) .* wDot) + rateInt;

    lo = zeros(6, 1); lo(reversible) = -1; hi = ones(6, 1);
    lo(~active) = 0; hi(~active) = 0;
    u = allocate(M, [tau; T], lo, hi);
    uOut = u;
    if restoring, uOut(1) = sRest * u(1); end
    uOut(~active) = 0;

    for i = 1:6
        tgt = uOut(i);
        if tgt < 0, tgt = max(tgt / D.rev, -1) * D.rev; end
        if ~active(i), tgt = 0;
        elseif ~reversible(i), tgt = max(tgt, D.idle);
        elseif abs(tgt) > 1e-3, tgt = sign(tgt) * max(abs(tgt), D.idle);
        end
        running = abs(uAct(i)) > 0.01;
        if running
            tStart(i) = NaN;
            if tgt * uAct(i) < 0, tgt = 0; end          % a reversal stops the motor first
        elseif abs(tgt) > 0.01
            if isnan(tStart(i)), tStart(i) = t; end
            if t - tStart(i) < D.tStart, tgt = 0; end
        else
            tStart(i) = NaN;
        end
        uAct(i) = uAct(i) + (tgt - uAct(i)) * dt / D.tauM;
    end

    Tr = D.Tmax * (1 + D.dT) .* uAct;
    tauPhys = [-rPhys(:, 2)' * Tr; rPhys(:, 1)' * Tr; D.kq * spin * Tr] - cross(D.cg, [0; 0; -sum(Tr)]);
    a = [0; 0; g] + Rb * [0; 0; -sum(Tr)] / D.m - 0.3 * v / D.m;
    wd = D.J \ (tauPhys - cross(w, D.J * w) - 0.002 * w);
    v = v + a * dt; p = p + v * dt;
    w = w + wd * dt;
    q = q + 0.5 * q_mul(q, [0; w]) * dt; q = q / norm(q);

    if t >= 0 && mod(n, 5) == 0
        k = k + 1;
        R.t(k) = t; R.eul(k, :) = rad2deg(euler(q)); R.w(k, :) = rad2deg(w');
        R.v(k, :) = v'; R.h(k) = -p(3);
        uLog = uOut'; uLog(~active) = NaN; R.u(k, :) = uLog;
    end
end
for c = {'t', 'eul', 'w', 'v', 'h', 'u'}, R.(c{1}) = R.(c{1})(1:k, :); end
end

function sc = px4_scale(Mx)
% ControlAllocationPseudoInverse normalisation: roll and pitch together, yaw separately, mean thrust
n = size(Mx, 1);
rp = max(sqrt(sum(Mx(:, 1).^2) / (n / 2)), sqrt(sum(Mx(:, 2).^2) / (n / 2)));
th = abs(Mx(:, 4)); th = mean(th(th > 1e-9));
sc = [rp rp max(Mx(:, 3)) th];
end

function u = allocate(M, c, lo, hi)
% ControlAllocationSequentialDesaturation with MC_AIRMODE 0
cNoYaw = c; cNoYaw(3) = 0;
u = M * cNoYaw;
u = desaturate(u, M(:, 4), lo, hi, true);
u = desaturate(u, M(:, 1), lo, hi, false);
u = desaturate(u, M(:, 2), lo, hi, false);
u = u + M(:, 3) * c(3);
u = desaturate(u, M(:, 3), lo, hi + 0.15 * (hi - lo), false);
u = desaturate(u, M(:, 4), lo, hi, true);
u = min(max(u, lo), hi);
end

function u = desaturate(u, d, lo, hi, reduceOnly)
for step = [1 0.5]
    k = step * desat_gain(d, u, lo, hi);
    if reduceOnly, k = min(k, 0); end
    u = u + k * d;
end
end

function k = desat_gain(d, u, lo, hi)
kMin = 0; kMax = 0;
for i = 1:numel(u)
    if abs(d(i)) < 1e-9, continue; end
    if u(i) < lo(i), kk = (lo(i) - u(i)) / d(i); kMin = min(kMin, kk); kMax = max(kMax, kk); end
    if u(i) > hi(i), kk = (hi(i) - u(i)) / d(i); kMin = min(kMin, kk); kMax = max(kMax, kk); end
end
k = kMin + kMax;
end

function Rm = rotation(q)
w = q(1); x = q(2); y = q(3); z = q(4);
Rm = [1 - 2 * (y^2 + z^2), 2 * (x * y - w * z), 2 * (x * z + w * y);
      2 * (x * y + w * z), 1 - 2 * (x^2 + z^2), 2 * (y * z - w * x);
      2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x^2 + y^2)];
end

function r = q_mul(a, b)
r = [a(1) * b(1) - a(2:4)' * b(2:4); a(1) * b(2:4) + b(1) * a(2:4) + cross(a(2:4), b(2:4))];
end

function r = q_conj(q), r = [q(1); -q(2:4)]; end

function e = euler(q)
w = q(1); x = q(2); y = q(3); z = q(4);
e = [atan2(2 * (w * x + y * z), 1 - 2 * (x^2 + y^2)), asin(max(-1, min(1, 2 * (w * y - z * x)))), ...
     atan2(2 * (w * z + x * y), 1 - 2 * (y^2 + z^2))];
end

function bands(ax, S)
xregion(ax, S.tStop, S.tRelease, 'FaceColor', palette().ink2, 'FaceAlpha', 0.12, 'HandleVisibility', 'off');
xregion(ax, S.tRelease, S.tRelease + S.spin + S.blend, 'FaceColor', palette().ink2, 'FaceAlpha', 0.05, 'HandleVisibility', 'off');
end
