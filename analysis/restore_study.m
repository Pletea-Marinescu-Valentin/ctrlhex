function restore_study(nMonteCarlo)
% Studies with the restore model of restore_simulation:
%   ablation   one-step, column only, output only, column and output, plus the idle compensation
%   timing     peak roll over the spin-up time, the blend time and the motor dead time (large vehicle)
%   Monte Carlo  random CG offset (|x|, |y| <= 5 mm), thrust mismatch (<= 3 %) and gyro noise seed
% Writes results/restore_study.txt and the Monte Carlo figure results/figures/paper/restore_study.{pdf,png}.

if nargin < 1, nMonteCarlo = 40; end
root = fileparts(fileparts(mfilename('fullpath')));
txt = fopen(fullfile(root, 'results', 'restore_study.txt'), 'w');
vehicles = {'demonstrator', 'small'};

%% Ablation (peak roll deg / roll rate deg/s / lateral speed m/s in the 3 s after the release)
emit(txt, 'Ablation: peak roll (deg) / roll rate (deg/s) / lateral speed (m/s) after the restore\n');
cases = {'column and output', struct('variant', 'both'); 'column only', struct('variant', 'column'); ...
         'output only', struct('variant', 'output'); 'column and output + idle compensation', ...
         struct('variant', 'both', 'idleComp', true)};
for v = 1:2
    emit(txt, '  %s\n', vehicles{v});
    for c = 1:size(cases, 1)
        P = restore_simulation(vehicles{v}, false, cases{c, 2});
        if c == 1, emit(txt, '    %-38s %5.1f / %5.0f / %4.2f\n', 'one-step (PX4)', P(1, :)); end
        emit(txt, '    %-38s %5.1f / %5.0f / %4.2f\n', cases{c, 1}, P(2, :));
    end
end

%% Timing (large vehicle)
spins = [0 0.1 0.2 0.3 0.5]; blends = [0.25 0.5 1 2]; deads = [0.05 0.10 0.15 0.25];
comp = [false true];
rollSB = zeros(numel(spins), numel(blends), 2); rollDS = zeros(numel(deads), numel(spins), 2);
rollDS1 = zeros(numel(deads), 1);
for k = 1:2
    for a = 1:numel(spins)
        for b = 1:numel(blends)
            P = restore_simulation('demonstrator', false, struct('spin', spins(a), 'blend', blends(b), 'idleComp', comp(k)));
            rollSB(a, b, k) = P(2, 1); oneStep = P(1, 1);
        end
        for d = 1:numel(deads)
            P = restore_simulation('demonstrator', false, struct('spin', spins(a), 'tStart', deads(d), 'idleComp', comp(k)));
            rollDS(d, a, k) = P(2, 1); rollDS1(d) = P(1, 1);
        end
    end
end
names = {'without', 'with'};
for k = 1:2
    emit(txt, '\nTiming, large vehicle, %s idle compensation: peak roll (deg); one-step %.1f deg (dead time 0.15 s)\n', names{k}, oneStep);
    emit(txt, '  spin-up / blend  %s\n', sprintf('%7.2f s', blends));
    for a = 1:numel(spins), emit(txt, '  %6.2f s        %s\n', spins(a), sprintf('%9.1f', rollSB(a, :, k))); end
    emit(txt, '  dead time / spin-up (blend 1 s) %s   one-step\n', sprintf('%7.2f s', spins));
    for d = 1:numel(deads), emit(txt, '  %6.2f s           %s %9.1f\n', deads(d), sprintf('%9.1f', rollDS(d, :, k)), rollDS1(d)); end
end

%% Monte Carlo
emit(txt, '\nMonte Carlo, %d vehicles each: peak roll (deg) median / 90th percentile / max\n', nMonteCarlo);
rng(1);
labels = {'one-step', 'gradual', 'gradual + idle compensation'};
MC = cell(1, 2);
for v = 1:2
    R = zeros(nMonteCarlo, 3);
    for n = 1:nMonteCarlo
        o = struct('cg', [0.010 * (rand(2, 1) - 0.5); 0], 'dT', 0.06 * (rand(6, 1) - 0.5), 'seed', n);
        P = restore_simulation(vehicles{v}, false, o);
        o.idleComp = true;
        Q = restore_simulation(vehicles{v}, false, o);
        R(n, :) = [P(:, 1)' Q(2, 1)];
    end
    for c = 1:3
        r = sort(R(:, c));
        emit(txt, '  %-13s %-28s %5.1f / %5.1f / %5.1f\n', vehicles{v}, labels{c}, median(r), r(ceil(0.9 * numel(r))), r(end));
    end
    emit(txt, '  %-13s below one-step: gradual %d, with idle compensation %d of %d cases\n', vehicles{v}, ...
         sum(R(:, 2) < R(:, 1)), sum(R(:, 3) < R(:, 1)), nMonteCarlo);
    MC{v} = R;
end
fclose(txt);

%% Figure: Monte Carlo distribution of the peak roll after the restore
f = figure('Color', 'w', 'Units', 'inches', 'Position', [1 1 3.5 2.4]);
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
col = {palette().orange, palette().blue, palette().ink};
sty = {'-', '-', '--'};
titles = {'Large hexacopter', 'Small hexacopter'};
top = max(cellfun(@(R) max(R(:)), MC));
for v = 1:2
    ax = nexttile(tl); hold(ax, 'on');
    h = gobjects(1, 3);
    for c = 1:3
        r = sort(MC{v}(:, c));
        h(c) = stairs(ax, [0; r], (0:numel(r))' / numel(r), sty{c}, 'Color', col{c}, 'LineWidth', 1);
    end
    title(ax, titles{v}, 'FontWeight', 'normal');
    xlabel(ax, 'Peak roll (deg)'); xlim(ax, [0 top * 1.05]); ylim(ax, [0 1]);
    if v == 1
        ylabel(ax, 'Fraction of vehicles');
        lg = legend(ax, h, {'one-step', 'gradual', 'gradual + idle comp.'}, 'Location', 'southeast', 'Box', 'off', ...
                    'FontSize', 6, 'FontName', 'Times New Roman', 'TextColor', palette().ink2);
        lg.ItemTokenSize = [10 6];
    end
    style_axes(ax); ax.FontName = 'Times New Roman'; ax.FontSize = 8; ax.LineWidth = 0.5;
    ax.XLabel.FontSize = 8; ax.YLabel.FontSize = 8; ax.Title.FontSize = 8;
end
save_figure(f, fullfile(root, 'results', 'figures', 'paper'), 'restore_study', true);
close(f);
end

function emit(txt, varargin)
% print to the console and to the summary file
fprintf(varargin{:});
fprintf(txt, varargin{:});
end
