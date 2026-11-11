function s = yig_plot_results(out)
%YIG_PLOT_RESULTS  Plot a run from either the Simulink model or the script.
%   s = yig_plot_results(out)
%     out : Simulink.SimulationOutput from sim('yig_tuning_model')
%           or the struct returned by yig_sim_script()
%     s   : the run as a plain struct (t, ref, u, y, truth)
%
%   truth columns:  1 I_coil  2 Vc  3 busy  4 ready  5 I_trim
%                   6 m  7 H_eff  8 H_coil  9 f0  10 B_gap  11 P_out_dBm

s = to_struct(out);
t = s.t*1e3;   % ms

figure('Name', 'YIG tuning run', 'Color', 'w');
ax(1) = subplot(4,1,1);
plot(t, s.ref/1e9, 'k--', t, s.truth(:,9)/1e9, 'b'); grid on
ylabel('f (GHz)'); legend('target', 'f_0 (true)', 'Location', 'best');
title('Filter centre frequency');

ax(2) = subplot(4,1,2);
err = (s.truth(:,9) - s.ref)/1e6;
plot(t, err, 'r'); grid on; ylim([-60 60]);
ylabel('error (MHz)'); title('Tuning error (clipped to \pm60 MHz)');

ax(3) = subplot(4,1,3);
plot(t, s.truth(:,1), 'b', t, s.truth(:,2)/50, 'Color', [0.85 0.4 0]); grid on
ylabel('A  |  V/50'); legend('I_{coil}', 'V_c / 50', 'Location', 'best');
title('Write pulses');

ax(4) = subplot(4,1,4);
yyaxis left;  plot(t, s.truth(:,5)); ylabel('I_{trim} (A)');
yyaxis right; plot(t, s.truth(:,6)); ylabel('m');
grid on; xlabel('time (ms)'); title('Trim current and magnet state');
linkaxes(ax, 'x');
end

%% ======================================================================
function s = to_struct(out)
if isstruct(out)
    s = out;
    return;
end
tr = out.get('log_truth');
s.t     = tr.Time(:);
s.truth = ts_data(tr);
s.y     = zoh(out.get('log_y'),   s.t);
s.u     = zoh(out.get('log_u'),   s.t);
s.ref   = zoh(out.get('log_ref'), s.t);
s.source = 2;   % 2 = Simulink
end

function d = ts_data(ts)
d = ts.Data;
if ndims(d) == 3                 % k x 1 x N  ->  N x k
    d = permute(d, [3 1 2]);
end
d = reshape(d, size(d,1), []);
end

function v = zoh(ts, t)
d  = ts_data(ts);
tt = ts.Time(:);
idx = zeros(numel(t), 1);
j = 1;
for k = 1:numel(t)
    while j < numel(tt) && tt(j+1) <= t(k) + 1e-12
        j = j + 1;
    end
    idx(k) = j;
end
v = d(idx, :);
end
