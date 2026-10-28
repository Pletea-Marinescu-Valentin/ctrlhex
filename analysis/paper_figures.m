function paper_figures()
% Figures of the conference paper in IEEE column format (single column 3.5 in, double column 7.16 in,
% Times 8 pt), written to results/figures/paper.
% Needs the flight logs (data/flights) and the SIH logs of sim/experiments.sh (results/sih).

root = fileparts(fileparts(mfilename('fullpath')));
outDir = fullfile(root, 'results', 'figures', 'paper');
if ~exist(outDir, 'dir'), mkdir(outDir); end

[Sd, Ld, rd] = flight_analysis('demonstrator', false);
[Ss, ~, rs] = flight_analysis('small', false);

flight_figure(Ld, outDir);
stop_figure(Ld, rd, outDir);
every_stop_figure({rd, rs}, {Sd, Ss}, {'Large hexacopter', 'Small hexacopter'}, outDir);
sih_restore_figure(fullfile(root, 'results', 'sih'), outDir);
close all
fprintf('paper figures in %s\n', outDir);
end

%% Whole flight of the large hexacopter: height, tilt, stopped motor
function flight_figure(L, outDir)
f = paper_figure(7.16, 3.1);
tl = tiledlayout(f, 3, 1, 'TileSpacing', 'tight', 'Padding', 'compact');
ax1 = nexttile(tl); hold(ax1, 'on'); stop_bands(ax1, L);
plot(ax1, L.lp_t, L.height, 'Color', palette().blue, 'LineWidth', 1);
yline(ax1, 1.25, '--', 'ceiling', 'Color', palette().muted, 'LabelHorizontalAlignment', 'left', 'FontSize', 7, ...
      'FontName', 'Times New Roman', 'LabelColor', palette().ink2);
yline(ax1, 1.50, ':', 'geofence', 'Color', palette().muted, 'LabelHorizontalAlignment', 'left', 'FontSize', 7, ...
      'FontName', 'Times New Roman', 'LabelColor', palette().ink2);
ylabel(ax1, 'Height (m)'); ylim(ax1, [-0.1 1.75]);
ax2 = nexttile(tl); hold(ax2, 'on'); stop_bands(ax2, L);
h1 = plot(ax2, L.att_t, L.tilt, 'Color', palette().blue, 'LineWidth', 1);
h2 = plot(ax2, L.att_t, L.tilt_sp, '--', 'Color', palette().orange, 'LineWidth', 0.8);
ylabel(ax2, 'Tilt (deg)');
paper_legend(ax2, [h1 h2], {'measured', 'commanded'}, 'northeast');
ax3 = nexttile(tl); hold(ax3, 'on');
for e = L.ev
    if ~e.applied, continue; end
    rectangle(ax3, 'Position', [e.t_stop, e.motor - 0.32, e.t_restore - e.t_stop, 0.64], ...
              'FaceColor', palette().ink2, 'EdgeColor', 'none');
    rectangle(ax3, 'Position', [e.t_restore, e.motor - 0.32, max(e.t_done - e.t_restore, 0.01), 0.64], ...
              'FaceColor', palette().orange, 'EdgeColor', 'none');
end
set(ax3, 'YDir', 'reverse', 'YLim', [0.4 6.6], 'YTick', 1:6, 'YTickLabel', compose('M%d', 1:6));
ylabel(ax3, 'Stopped motor');
hs = [patch(ax3, NaN, NaN, 'k', 'FaceColor', palette().ink2, 'EdgeColor', 'none'), ...
      patch(ax3, NaN, NaN, 'k', 'FaceColor', palette().orange, 'EdgeColor', 'none')];
paper_legend(ax3, hs, {'stopped', 'gradual restore'}, 'northeast');
xlabel(tl, 'Time since arming (s)', 'FontName', 'Times New Roman', 'FontSize', 8, 'Color', palette().ink2);
for ax = [ax1 ax2 ax3], paper_axes(ax); xlim(ax, [0 L.duration]); end
ax1.XTickLabel = {}; ax2.XTickLabel = {};
linkaxes([ax1 ax2 ax3], 'x');
save_figure(f, outDir, 'flight_large', true);
end

%% A typical stop of the large hexacopter (median stopped-phase error, no heading reset)
function stop_figure(L, res, outDir)
cand = res(res(:, 5) == 0, :);
[~, ord] = sort(cand(:, 3));
x = L.ev(cand(ord(ceil(end / 2)), 1));
m = x.motor; o = L.opposite(m);
t1 = x.t_stop - 1; t2 = min(x.t_done + 1.5, L.duration);
f = paper_figure(3.5, 3.6);
tl = tiledlayout(f, 3, 1, 'TileSpacing', 'tight', 'Padding', 'compact');
ax1 = nexttile(tl); hold(ax1, 'on'); event_bands(ax1, x);
h = [plot(ax1, L.att_t, L.eul(:, 1), 'Color', palette().blue, 'LineWidth', 1), ...
     plot(ax1, L.att_t, L.eul(:, 2), 'Color', palette().orange, 'LineWidth', 1)];
plot(ax1, L.att_t, L.eul_sp(:, 1), '--', 'Color', palette().blue, 'LineWidth', 0.6);
plot(ax1, L.att_t, L.eul_sp(:, 2), '--', 'Color', palette().orange, 'LineWidth', 0.6);
ylabel(ax1, 'Angle (deg)');
k = L.att_t >= t1 & L.att_t <= t2;
headroom(ax1, [L.eul(k, 1:2); L.eul_sp(k, 1:2)]);
paper_legend(ax1, h, {'roll', 'pitch'}, 'northwest');
axs = gobjects(1, 2);
for p = 1:2
    ax = nexttile(tl); hold(ax, 'on'); event_bands(ax, x);
    if p == 1, tt = L.mot_t; y = L.mot; yl = 'Command';
    else, tt = L.esc_t; y = L.rpm; yl = 'Speed (krpm)'; end
    hg = plot(ax, tt, y(:, setdiff(1:6, [m o])), 'Color', palette().muted, 'LineWidth', 0.6);
    ho = plot(ax, tt, y(:, o), '-.', 'Color', palette().orange, 'LineWidth', 1);
    hm = plot(ax, tt, y(:, m), 'Color', palette().blue, 'LineWidth', 1);
    yline(ax, 0, 'Color', palette().base, 'LineWidth', 0.5);
    ylabel(ax, yl);
    if p == 1
        headroom(ax, y(tt >= t1 & tt <= t2, :));
        paper_legend(ax, [hm ho hg(1)], {sprintf('M%d stopped', m), sprintf('M%d opposite', o), 'other four'}, 'northwest');
    end
    axs(p) = ax;
end
xlabel(tl, 'Time since arming (s)', 'FontName', 'Times New Roman', 'FontSize', 8, 'Color', palette().ink2);
for ax = [ax1 axs], paper_axes(ax); xlim(ax, [t1 t2]); end
ax1.XTickLabel = {}; axs(1).XTickLabel = {};
linkaxes([ax1 axs], 'x');
save_figure(f, outDir, 'one_stop_large', true);
fprintf('typical stop: motor %d, opposite %d, stopped %.1f s, restored in %.2f s, at t = %.1f s\n', ...
        m, o, x.t_restore - x.t_stop, x.t_done - x.t_restore, x.t_stop);
end

%% Every stop of both vehicles against six-motor flight
function every_stop_figure(R, S, names, outDir)
f = paper_figure(3.5, 2.9);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for v = 1:2
    res = R{v}; n = size(res, 1);
    ax = nexttile(tl); hold(ax, 'on');
    b = bar(ax, 1:n, res(:, 3:4), 0.8, 'EdgeColor', 'none');
    b(1).FaceColor = palette().blue; b(2).FaceColor = palette().orange;
    hn = yline(ax, S{v}.tiltNominal(2), '--', 'Color', palette().ink2, 'LineWidth', 0.8);
    top = 14;
    for k = find(res(:, 5))'
        text(ax, k, min(max(res(k, 3:4)), top) + 0.3, '*', 'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'bottom', 'FontSize', 9, 'FontName', 'Times New Roman', 'Color', palette().ink2);
    end
    for k = find(res(:, 3) > top)'          % bars cut at the top of the axis carry their value
        text(ax, k - 0.45, top - 1, sprintf('%.1f', res(k, 3)), 'HorizontalAlignment', 'right', 'FontSize', 7, ...
             'FontName', 'Times New Roman', 'Color', palette().ink2);
    end
    set(ax, 'XTick', 1:n, 'XTickLabel', compose('M%d', res(:, 2)));
    ylim(ax, [0 top + 2.5]); xlim(ax, [0.4 n + 0.6]);
    ylabel(ax, 'Max tilt error (deg)');
    title(ax, sprintf('%s, %d stops', names{v}, n), 'FontWeight', 'normal');
    if v == 1
        lg = paper_legend(ax, [b hn], {'motor stopped', 'restore', 'six motors, 95th pct.'}, 'north');
        lg.Layout.Tile = 'north';
    end
    paper_axes(ax); ax.XGrid = 'off';
    ax.Clipping = 'on';
end
xlabel(tl, 'Stops in flight order (stopped motor)', 'FontName', 'Times New Roman', 'FontSize', 8, 'Color', palette().ink2);
save_figure(f, outDir, 'every_stop', true);
end

%% SIH, PX4 firmware: one-step against gradual restore of a stopped motor (Altitude mode)
function sih_restore_figure(sihDir, outDir)
runs = {'restore_alt_one_step', 'restore_alt_gradual'};
f = paper_figure(3.5, 3.1);
tl = tiledlayout(f, 3, 1, 'TileSpacing', 'tight', 'Padding', 'compact');
ax = gobjects(1, 3); for k = 1:3, ax(k) = nexttile(tl); hold(ax(k), 'on'); end
col = {palette().orange, palette().blue}; sty = {'-', '-'};
h = gobjects(1, 2);
for r = 1:2
    G = read_sih(fullfile(sihDir, [runs{r} '.ulg']));
    h(r) = plot(ax(1), G.att_t - G.t_restore, G.tilt_err, sty{r}, 'Color', col{r}, 'LineWidth', 1);
    plot(ax(2), G.w_t - G.t_restore, G.rate, sty{r}, 'Color', col{r}, 'LineWidth', 1);
    if r == 2
        o = G.opposite; m = G.motor;
        hg = plot(ax(3), G.mot_t - G.t_restore, G.mot(:, setdiff(1:6, [m o])), 'Color', palette().muted, 'LineWidth', 0.6);
        ho = plot(ax(3), G.mot_t - G.t_restore, G.mot(:, o), '-.', 'Color', palette().orange, 'LineWidth', 1);
        hm = plot(ax(3), G.mot_t - G.t_restore, G.mot(:, m), 'Color', palette().blue, 'LineWidth', 1);
        yline(ax(3), 0, 'Color', palette().base, 'LineWidth', 0.5);
        xregion(ax(3), 0, G.t_done - G.t_restore, 'FaceColor', palette().ink2, 'FaceAlpha', 0.07, 'HandleVisibility', 'off');
        k = G.mot_t > G.t_restore - 0.5 & G.mot_t < G.t_restore + 2.5;
        headroom(ax(3), G.mot(k, :));
        paper_legend(ax(3), [hm ho hg(1)], {'restored motor', 'opposite', 'other four'}, 'northeast');
    end
end
ylabel(ax(1), 'Tilt error (deg)'); ylabel(ax(2), 'Rate (deg/s)'); ylabel(ax(3), 'Command (gradual)');
paper_legend(ax(1), h, {'one-step restore (PX4)', 'gradual restore'}, 'northeast');
xlabel(tl, 'Time from the restore command (s)', 'FontName', 'Times New Roman', 'FontSize', 8, 'Color', palette().ink2);
for k = 1:3, paper_axes(ax(k)); xlim(ax(k), [-0.5 2.5]); end
ax(1).XTickLabel = {}; ax(2).XTickLabel = {};
linkaxes(ax, 'x');
save_figure(f, outDir, 'sih_restore', true);
end

function G = read_sih(file)
u = ulogreader(file);
t0 = u.StartTime;
msg = readLoggedOutput(u);
if istimetable(msg), tm = seconds(msg.Properties.RowTimes - t0); else, tm = seconds(msg.timestamp - t0); end
txt = string(msg.Messages);
G.t_restore = tm(find(contains(txt, 'Restoring motors'), 1));
G.t_done = tm(find(contains(txt, 'All motors restored') & tm >= G.t_restore, 1));
tok = regexp(txt(find(contains(txt, 'Restoring motors'), 1)), 'mask 0x([0-9a-fA-F]+)', 'tokens', 'once');
G.motor = log2(hex2dec(tok(1))) + 1;
[att, G.att_t] = topic(u, 'vehicle_attitude', t0);
[attsp, attsp_t] = topic(u, 'vehicle_attitude_setpoint', t0);
[w, G.w_t] = topic(u, 'vehicle_angular_velocity', t0);
[mot, G.mot_t] = topic(u, 'actuator_motors', t0);
qsp = interp1(attsp_t, attsp.q_d, G.att_t, 'previous', NaN);
zb = body_z(att.q); zsp = body_z(qsp);
G.tilt_err = acosd(min(1, max(-1, sum(zb .* zsp, 2))));
G.rate = rad2deg(vecnorm(double(w.xyz), 2, 2));
G.mot = double(mot.control(:, 1:6));
% opposite rotor: the running motor with the smallest mean command in the second before the restore
k = G.mot_t > G.t_restore - 1 & G.mot_t < G.t_restore;
mc = mean(abs(G.mot(k, :)), 1, 'omitnan'); mc(G.motor) = Inf;
[~, G.opposite] = min(mc);
end

function [tt, t] = topic(u, name, t0)
d = readTopicMsgs(u, 'TopicNames', {name}, 'InstanceID', {0});
tt = d.TopicMessages{1};
t = seconds(tt.Properties.RowTimes - t0);
end

function zb = body_z(q)
w = q(:, 1); x = q(:, 2); y = q(:, 3); z = q(:, 4);
zb = [2 * (x .* z + w .* y), 2 * (y .* z - w .* x), 1 - 2 * (x.^2 + y.^2)];
end

function headroom(ax, y)
% y limits from the plotted window, with room above the data for a legend
lo = min(y, [], 'all'); hi = max(y, [], 'all'); pad = 0.08 * (hi - lo);
ylim(ax, [lo - pad, hi + 0.5 * (hi - lo)]);
end

function f = paper_figure(widthIn, heightIn)
f = figure('Color', 'w', 'Units', 'inches', 'Position', [1 1 widthIn heightIn]);
end

function paper_axes(ax)
style_axes(ax);
ax.FontName = 'Times New Roman'; ax.FontSize = 8; ax.LineWidth = 0.5;
ax.Title.FontSize = 8;
ax.XLabel.FontSize = 8; ax.YLabel.FontSize = 8;
end

function lg = paper_legend(ax, h, labels, loc)
lg = legend(ax, h, labels, 'Location', loc, 'Box', 'off', 'FontName', 'Times New Roman', 'FontSize', 7, ...
            'TextColor', palette().ink2, 'Orientation', 'horizontal');
lg.ItemTokenSize = [14 6];
end

function stop_bands(ax, L)
for e = L.ev
    if e.applied, event_bands(ax, e); end
end
end

function event_bands(ax, e)
if e.t_restore > e.t_stop
    xregion(ax, e.t_stop, e.t_restore, 'FaceColor', palette().ink2, 'FaceAlpha', 0.14, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end
if e.t_done > e.t_restore
    xregion(ax, e.t_restore, e.t_done, 'FaceColor', palette().orange, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end
end
