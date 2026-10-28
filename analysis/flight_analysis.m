function S = flight_analysis(vehicle, saveFigures)
% Flight-test analysis of commanded motor stops.
%   S = flight_analysis('demonstrator')       large hexacopter, 2026-10-21
%   S = flight_analysis('small')              small hexacopter, 2026-10-14
% Tilt error = angle between the commanded and the measured body z axis.

if nargin < 1, vehicle = 'demonstrator'; end
if nargin < 2, saveFigures = true; end

root = fileparts(fileparts(mfilename('fullpath')));
switch vehicle
    case 'demonstrator'
        logFile = fullfile(root, 'data', 'flights', 'demonstrator_2026-10-21.ulg');
        heightTitle = 'Height above ground (EKF, range sensor)';
        figTitle = 'Large hexacopter flight';
    case 'small'
        logFile = fullfile(root, 'data', 'flights', 'small_hexacopter_2026-10-14.ulg');
        heightTitle = 'Height above the arming point (EKF, barometer)';
        figTitle = 'Small hexacopter flight';
    otherwise
        error('vehicle must be ''demonstrator'' or ''small''');
end
ceiling  = 1.25;
geofence = 1.50;
outDir = fullfile(root, 'results', 'figures', vehicle);
if saveFigures && ~exist(outDir, 'dir'), mkdir(outDir); end

L = read_log(logFile);
ev = L.ev;
applied = find([ev.applied]);

% [stop, motor, max error stopped, max error restoring, EKF yaw reset]
res = zeros(0, 5);
for e = applied
    x = ev(e);
    inStop = L.att_t >= x.t_stop & L.att_t <= x.t_restore;
    inRest = L.att_t >= x.t_restore & L.att_t <= x.t_done + 0.5;
    yawReset = any(L.yawReset >= x.t_stop & L.yawReset <= x.t_done + 0.5);
    res(end + 1, :) = [e, x.motor, max(L.tilt_err(inStop)), max(L.tilt_err(inRest)), yawReset]; %#ok<AGROW>
end
restoreTime = median([ev(applied).t_done] - [ev(applied).t_restore]);

%% Figure 1: the whole flight
f = new_figure(17, 15);
tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = nexttile(tl); hold(ax1, 'on');
stop_bands(ax1, L);
plot_height(ax1, L, ceiling, geofence);
title(ax1, heightTitle);
ax2 = nexttile(tl); hold(ax2, 'on');
stop_bands(ax2, L);
plot(ax2, L.att_t, L.tilt, 'Color', palette().blue, 'LineWidth', 1.5);
plot(ax2, L.att_t, L.tilt_sp, '--', 'Color', palette().orange, 'LineWidth', 1.25);
ylabel(ax2, 'Tilt [°]'); title(ax2, 'Tilt: commanded by the pilot and measured');
add_legend(ax2, {'measured', 'commanded (setpoint)'});
ax3 = nexttile(tl); hold(ax3, 'on');
timeline(ax3, L);
title(ax3, 'Which motor is stopped, and when');
set_title(tl, sprintf('%s: %d stops covering all six motors in %.0f s of flight', figTitle, numel(applied), L.airborne));
xlabel(tl, 'Time since arming [s]', 'Color', palette().ink2, 'FontName', plot_font(), 'FontSize', 9);
for ax = [ax1 ax2 ax3], style_axes(ax); xlim(ax, [0 L.duration]); end
linkaxes([ax1 ax2 ax3], 'x');
save_figure(f, outDir, 'flight', saveFigures);

%% Figure 2: a typical stop (median error, no EKF yaw reset)
cand = res(res(:, 5) == 0, :);
[~, ord] = sort(cand(:, 3));
eSel = cand(ord(ceil(end / 2)), 1);
x = ev(eSel); m = x.motor; o = L.opposite(m);
t1 = x.t_stop - 1.5; t2 = min(x.t_done + 1.5, L.duration);
f = new_figure(17, 15);
tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = nexttile(tl); hold(ax1, 'on');
event_bands(ax1, x);
plot(ax1, L.att_t, L.eul(:, 1), 'Color', palette().blue, 'LineWidth', 1.5);
plot(ax1, L.att_t, L.eul(:, 2), 'Color', palette().orange, 'LineWidth', 1.5);
plot(ax1, L.att_t, L.eul_sp(:, 1), '--', 'Color', palette().blue, 'LineWidth', 0.9);
plot(ax1, L.att_t, L.eul_sp(:, 2), '--', 'Color', palette().orange, 'LineWidth', 0.9);
ylabel(ax1, 'Angle [°]'); xlim(ax1, [t1 t2]);
add_legend(ax1, {'roll', 'pitch', 'roll commanded', 'pitch commanded'}, [], 'tile-north');
axs = gobjects(1, 2);
for p = 1:2
    ax = nexttile(tl); hold(ax, 'on');
    event_bands(ax, x);
    if p == 1, tt = L.mot_t; y = L.mot; yl = 'Command [-1..1]';
    else, tt = L.esc_t; y = L.rpm; yl = 'Speed [×1000 rpm]'; end
    hg = plot(ax, tt, y(:, setdiff(1:6, [m o])), 'Color', palette().muted, 'LineWidth', 0.9);
    ho = plot(ax, tt, y(:, o), 'Color', palette().orange, 'LineWidth', 1.5);
    hm = plot(ax, tt, y(:, m), 'Color', palette().blue, 'LineWidth', 1.5);
    yline(ax, 0, 'Color', palette().base, 'LineWidth', 0.75);
    ylabel(ax, yl); xlim(ax, [t1 t2]);
    if p == 1
        add_legend(ax, {sprintf('M%d stopped', m), sprintf('M%d opposite (reversible)', o), 'other 4 motors'}, ...
                   [hm ho hg(1)], 'tile-south');
    end
    axs(p) = ax;
end
set_title(tl, sprintf('A typical stop: motor %d (%s), stopped for %.1f s, then restored gradually in %.1f s', ...
          m, L.position{m}, x.t_restore - x.t_stop, x.t_done - x.t_restore));
xlabel(tl, 'Time since arming [s]', 'Color', palette().ink2, 'FontName', plot_font(), 'FontSize', 9);
for ax = [ax1 axs], style_axes(ax); xlim(ax, [t1 t2]); end
linkaxes([ax1 axs], 'x');
save_figure(f, outDir, 'one_stop', saveFigures);

%% Figure 3: every stop
n = size(res, 1);
f = new_figure(17, 8.5);
tl = tiledlayout(f, 1, 1, 'Padding', 'compact');
ax = nexttile(tl); hold(ax, 'on');
b = bar(ax, 1:n, res(:, 3:4), 0.8, 'EdgeColor', 'none');
b(1).FaceColor = palette().blue; b(2).FaceColor = palette().orange;
for k = find(res(:, 5))'
    text(ax, k, max(res(k, 3:4)) + 0.3, '*', 'HorizontalAlignment', 'center', 'FontSize', 12, 'Color', palette().ink2);
end
set(ax, 'XTick', 1:n, 'XTickLabel', arrayfun(@(k) sprintf('M%d', res(k, 2)), 1:n, 'UniformOutput', false));
ylabel(ax, 'Max tilt error [°]');
lab = 'Stops in flight order (stopped motor)';
if any(res(:, 5)), lab = {lab, '* EKF heading reset (compass) during this stop'}; end
xlabel(ax, lab);
ylim(ax, [0 max(res(:, 3:4), [], 'all') + 1.5]);
add_legend(ax, {'motor stopped', sprintf('during the restore (%.1f s)', restoreTime)}, b, 'tile-north');
set_title(tl, sprintf('Every stop: median tilt error %.1f° with the motor stopped, %.1f° during the restore', ...
          median(res(:, 3)), median(res(:, 4))));
style_axes(ax); ax.XGrid = 'off';
save_figure(f, outDir, 'every_stop', saveFigures);

%% Numbers
S = summary(L, res, restoreTime, numel(ev), vehicle);
fprintf('\n%s (%s)\n', vehicle, logFile);
fprintf('  airborne time                      %.0f s\n', L.airborne);
fprintf('  stops applied / requested          %d / %d, motors covered: %s\n', S.applied, S.requested, mat2str(S.motors));
fprintf('  restore duration (median)          %.1f s\n', S.restoreTime);
fprintf('  tilt error stopped, median / max   %.1f / %.1f deg\n', S.tiltStopped);
fprintf('  tilt error restoring, median / max %.1f / %.1f deg\n', S.tiltRestore);
fprintf('  hover command -> four loaded rotors (median)  %.2f -> %.2f\n', S.hoverCmd, S.loadedCmd);
fprintf('  opposite rotor: median command %.2f, in reverse %.0f%% of the time\n', S.oppositeCmd, 100 * S.oppositeReverse);
fprintf('  max EKF height %.2f m (%s)\n', S.maxHeight, S.heightSource);
if isfinite(S.maxDistance)
    fprintf('  max horizontal distance from arming point %.2f m\n', S.maxDistance);
end
fprintf('  battery voltage %.1f -> %.1f V\n', S.voltage);
if saveFigures, fprintf('  figures in %s\n', outDir); end
end

function S = summary(L, res, restoreTime, requested, vehicle)
S.airborne = L.airborne;
S.requested = requested;
S.applied = size(res, 1);
S.motors = unique(res(:, 2))';
S.restoreTime = restoreTime;
S.tiltStopped = [median(res(:, 3)) max(res(:, 3))];
S.tiltRestore = [median(res(:, 4)) max(res(:, 4))];

flying = L.mot_t > 0 & L.mot_t < L.duration & mean(L.mot, 2) > 0.1;
stopped = false(size(L.mot_t)); loaded = []; opp = [];
for e = L.ev([L.ev.applied])
    k = L.mot_t >= e.t_stop & L.mot_t <= e.t_restore;
    stopped = stopped | (L.mot_t >= e.t_stop & L.mot_t <= e.t_done + 0.5);
    others = setdiff(1:6, [e.motor L.opposite(e.motor)]);
    loaded = [loaded; reshape(L.mot(k, others), [], 1)]; %#ok<AGROW>
    opp = [opp; L.mot(k, L.opposite(e.motor))]; %#ok<AGROW>
end
S.hoverCmd = median(mean(L.mot(flying & ~stopped, :), 2));
S.loadedCmd = median(loaded);
S.oppositeCmd = median(opp);
S.oppositeReverse = mean(opp < 0);

inFlight = L.lp_t > 0 & L.lp_t < L.duration;
S.maxHeight = max(L.height(inFlight));
if strcmp(vehicle, 'demonstrator'), S.heightSource = 'range sensor'; else, S.heightSource = 'barometer'; end
k = inFlight & L.xy_valid;
if any(k)
    S.maxDistance = max(hypot(L.x(k) - L.x(find(k, 1)), L.y(k) - L.y(find(k, 1))));
else
    S.maxDistance = NaN;
end
v = L.voltage(L.bat_t > 0 & L.bat_t < L.duration);
S.voltage = [max(v) min(v)];
end

function L = read_log(file)
u = ulogreader(file);
t0 = u.StartTime;
L.duration = seconds(u.EndTime - t0);
[att, L.att_t]   = topic(u, 'vehicle_attitude', t0);
[attsp, attsp_t] = topic(u, 'vehicle_attitude_setpoint', t0);
[mot, L.mot_t]   = topic(u, 'actuator_motors', t0);
[esc, L.esc_t]   = topic(u, 'esc_status', t0);
[lp, L.lp_t]     = topic(u, 'vehicle_local_position', t0);
[bat, L.bat_t]   = topic(u, 'battery_status', t0);
[ld, ld_t]       = topic(u, 'vehicle_land_detected', t0);
air = find(~ld.landed);
L.airborne = ld_t(min(air(end) + 1, numel(ld_t))) - ld_t(air(1));

% stop at disarm: afterwards the estimator has no thrust and the baro height drifts
msg = readLoggedOutput(u);
if istimetable(msg), tm = seconds(msg.Properties.RowTimes - t0); else, tm = seconds(msg.timestamp - t0); end
iDis = find(contains(string(msg.Messages), 'Disarmed'), 1, 'last');
if ~isempty(iDis), L.duration = min(L.duration, tm(iDis)); end

q = att.q;
qd = attsp.q_d;
qd(vecnorm(qd, 2, 2) < 0.5 | -attsp.thrust_body(:, 3) < 0.02, :) = NaN;   % no thrust: setpoint meaningless
qsp = interp1(attsp_t, qd, L.att_t, 'previous', NaN);
L.eul = euler(q); L.eul_sp = euler(qsp);
L.eul(:, 3) = unwrap_deg(L.eul(:, 3));
L.eul_sp(:, 3) = unwrap_deg(L.eul_sp(:, 3));
zb = body_z(q); zsp = body_z(qsp);
L.tilt = acosd(clip1(zb(:, 3)));
L.tilt_sp = acosd(clip1(zsp(:, 3)));
L.tilt_err = acosd(clip1(sum(zb .* zsp, 2)));

L.mot = mot.control(:, 1:6);
L.rpm = zeros(numel(L.esc_t), 6);
for k = 1:6, L.rpm(:, k) = double(esc.(sprintf('esc[%d].esc_rpm', k - 1))) / 1000; end

L.height = double(lp.z(1) - lp.z);
L.x = double(lp.x); L.y = double(lp.y);
L.xy_valid = logical(lp.xy_valid);
L.voltage = double(bat.voltage_v);

% airframe 6001 (hexarotor X)
L.position = {'right', 'left', 'front-left', 'rear-right', 'front-right', 'rear-left'};
L.opposite = [2 1 4 3 6 5];

L.ev = stop_events(u, t0, L.duration);
L.yawReset = L.att_t([false; diff(double(att.quat_reset_counter)) ~= 0]);
end

function ev = stop_events(u, t0, tEnd)
% A stop lasts from "Injected" to the allocator's "Restoring motors"; the restore ends at "All motors restored".
% The allocator handles one failed motor: a request for a second motor is ignored (applied = false).
msg = readLoggedOutput(u);
if istimetable(msg), tm = seconds(msg.Properties.RowTimes - t0); else, tm = seconds(msg.timestamp - t0); end
txt = string(msg.Messages);
inj = zeros(0, 2); clr = {}; rest = zeros(0, 2); done = [];
for i = 1:numel(txt)
    tok = regexp(txt(i), 'Injected: motor off, instances ([\d,]+)', 'tokens', 'once');
    if ~isempty(tok), for m = str2double(split(tok(1), ','))', inj(end + 1, :) = [tm(i) m]; end, continue; end %#ok<AGROW>
    tok = regexp(txt(i), 'Cleared: motor, instances ([\d,]+)', 'tokens', 'once');
    if ~isempty(tok), clr(end + 1, :) = {tm(i), str2double(split(tok(1), ','))'}; continue; end %#ok<AGROW>
    tok = regexp(txt(i), 'Restoring motors \(mask 0x([0-9a-fA-F]+)\)', 'tokens', 'once');
    if ~isempty(tok), rest(end + 1, :) = [tm(i) hex2dec(tok(1))]; continue; end %#ok<AGROW>
    if contains(txt(i), 'All motors restored'), done(end + 1) = tm(i); end %#ok<AGROW>
end
ev = struct('motor', {}, 't_stop', {}, 't_release', {}, 't_restore', {}, 't_done', {}, 'applied', {});
for k = 1:size(inj, 1)
    t = inj(k, 1); m = inj(k, 2);
    tNext = min([inj(inj(:, 1) > t & inj(:, 2) == m, 1); tEnd]);
    e = struct('motor', m, 't_stop', t, 't_release', tNext, 't_restore', NaN, 't_done', NaN, 'applied', true);
    for j = 1:size(clr, 1)
        if clr{j, 1} > t && any(clr{j, 2} == m), e.t_release = min(clr{j, 1}, tNext); break; end
    end
    r = rest(rest(:, 1) >= e.t_release & rest(:, 1) < tNext & bitand(rest(:, 2), 2^(m - 1)) > 0, 1);
    if ~isempty(r)
        e.t_restore = r(1);
        g = done(done >= r(1) & done < tNext);
        if isempty(g), e.t_done = e.t_restore; else, e.t_done = g(1); end
    elseif isempty(rest)
        e.t_restore = e.t_release; e.t_done = e.t_release;
    else
        e.applied = false; e.t_restore = e.t_release; e.t_done = e.t_release;
    end
    ev(end + 1) = e; %#ok<AGROW>
end
end

function [tt, t] = topic(u, name, t0)
d = readTopicMsgs(u, 'TopicNames', {name}, 'InstanceID', {0});
tt = d.TopicMessages{1};
t = seconds(tt.Properties.RowTimes - t0);
end

function y = unwrap_deg(y)
ok = isfinite(y);
y(ok) = rad2deg(unwrap(deg2rad(y(ok))));
end

function e = euler(q)
w = q(:, 1); x = q(:, 2); y = q(:, 3); z = q(:, 4);
e = rad2deg([atan2(2 * (w .* x + y .* z), 1 - 2 * (x.^2 + y.^2)), ...
             asin(clip1(2 * (w .* y - z .* x))), ...
             atan2(2 * (w .* z + x .* y), 1 - 2 * (y.^2 + z.^2))]);
end

function x = clip1(x)
x = min(1, max(-1, x, 'includenan'), 'includenan');
end

function zb = body_z(q)
w = q(:, 1); x = q(:, 2); y = q(:, 3); z = q(:, 4);
zb = [2 * (x .* z + w .* y), 2 * (y .* z - w .* x), 1 - 2 * (x.^2 + y.^2)];
end

function stop_bands(ax, L)
for e = L.ev
    if e.applied, event_bands(ax, e); end
end
end

function event_bands(ax, e)
if e.applied
    band(ax, e.t_stop, e.t_restore, 0.14);
    band(ax, e.t_restore, e.t_done, 0.06);
else
    band(ax, e.t_stop, e.t_release, 0.04);
end
end

function band(ax, a, b, alpha)
if ~(b > a), return; end
xregion(ax, a, b, 'FaceColor', palette().ink2, 'FaceAlpha', alpha, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end

function timeline(ax, L)
for e = L.ev
    y = e.motor;
    if ~e.applied
        rectangle(ax, 'Position', [e.t_stop, y - 0.3, e.t_release - e.t_stop, 0.6], ...
                  'FaceColor', 'none', 'EdgeColor', palette().ink2, 'LineStyle', '--', 'LineWidth', 0.75, 'Curvature', 0.2);
        continue;
    end
    rectangle(ax, 'Position', [e.t_stop, y - 0.3, e.t_restore - e.t_stop, 0.6], ...
              'FaceColor', palette().ink2, 'EdgeColor', 'none', 'Curvature', 0.2);
    if e.t_done > e.t_restore
        rectangle(ax, 'Position', [e.t_restore, y - 0.3, e.t_done - e.t_restore, 0.6], ...
                  'FaceColor', palette().base, 'EdgeColor', 'none', 'Curvature', 0.2);
    end
end
set(ax, 'YDir', 'reverse', 'YLim', [0.4 6.6], 'YTick', 1:6, ...
    'YTickLabel', arrayfun(@(m) sprintf('M%d %s', m, L.position{m}), 1:6, 'UniformOutput', false));
h = [patch(ax, NaN, NaN, 'k', 'FaceColor', palette().ink2, 'EdgeColor', 'none'), ...
     patch(ax, NaN, NaN, 'k', 'FaceColor', palette().base, 'EdgeColor', 'none')];
labels = {'stopped', 'gradual restore'};
if any(~[L.ev.applied])
    h(3) = patch(ax, NaN, NaN, 'k', 'FaceColor', 'none', 'EdgeColor', palette().ink2, 'LineStyle', '--');
    labels{3} = 'switch ignored (another motor was stopped)';
end
add_legend(ax, labels, h, 'southoutside');
end

function plot_height(ax, L, ceiling, geofence)
plot(ax, L.lp_t, L.height, 'Color', palette().blue, 'LineWidth', 1.5);
threshold(ax, L.duration, ceiling, sprintf('ceiling %.2f m', ceiling), 'bottom');
threshold(ax, L.duration, geofence, sprintf('geofence %.2f m', geofence), 'top');
lo = -0.3;
ylim(ax, [lo max(geofence + 0.25, max(L.height(L.lp_t <= L.duration)) + 0.1)]);
if any(L.height(L.lp_t <= L.duration) < lo)
    text(ax, L.duration, lo, 'the baro estimate drops below 0 at landing ', 'FontSize', 7, 'FontName', plot_font(), ...
         'Color', palette().ink2, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
end
ylabel(ax, 'Height [m]');
end

function threshold(ax, duration, y, label, where)
yline(ax, y, '--', label, 'Color', palette().muted, 'LineWidth', 1, 'FontSize', 8, 'FontName', plot_font(), ...
      'LabelColor', palette().ink2, 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', where, ...
      'HandleVisibility', 'off');
xlim(ax, [0 duration]);
end
