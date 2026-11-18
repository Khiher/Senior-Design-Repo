function R = yig_spec_report(s, P, ref_profile, verbose)
%YIG_SPEC_REPORT  Check a run against the project specs.
%
%   R = yig_spec_report(s, P, ref_profile)
%     s : struct returned by yig_plot_results (works for script or Simulink)
%
%   For every target change in ref_profile:
%     response  time from the command until the TRUE centre frequency is
%               within spec_tol (10 MHz) and stays there for 1 ms
%               (spec: < 1 ms)
%     max err   worst TRUE error from that point to the next command,
%               including standby and wake-up re-tunes  (spec: <= 10 MHz)
%     out       total time out of spec after the response (a re-tune
%               forced by a disturbance shows up here as a short outage)
%     standby   average power while in standby/wake    (spec: < 1 mW)
%               = P_sleep + Hall power while it is on + trim I^2 R
%
%   The "true" error uses the plant's f0, not the controller's Hall-based
%   estimate, so sensor errors count against the spec.

if nargin < 4, verbose = true; end
tol = P.ctrl.spec_tol;
t = s.t;  f0 = s.truth(:,9);  ref = s.ref;
err = f0 - ref;
I_trim = s.truth(:,5);
hall_on = s.u(:,7);  mode = s.u(:,8);
dt = median(diff(t));
n_hold = max(1, round(1e-3/dt));
t_cmd = [ref_profile(:,1); t(end) + dt];

nseg = size(ref_profile, 1);
R.f_target = ref_profile(:,2);
R.t_resp = nan(nseg,1);  R.max_err = nan(nseg,1);  R.t_out = nan(nseg,1);
R.P_standby = nan(nseg,1);  R.standby_frac = zeros(nseg,1);
p_inst = P.pwr.P_sleep + hall_on*P.pwr.P_hall + I_trim.^2*P.pwr.R_trim;
sb_all = false(size(t));
for k = 1:nseg
    % stop 2 log samples short of the next command: Simulink's logged
    % reference can switch one sample early at the boundary
    idx = find(t >= t_cmd(k) & t < t_cmd(k+1) - 2*dt);
    if isempty(idx), continue; end
    ok = abs(err(idx)) <= tol;
    % first sample that starts a run of n_hold in-spec samples
    run = 0;  i_resp = [];
    for q = numel(ok):-1:1          % backwards: length of in-spec run from q
        if ok(q), run = run + 1; else, run = 0; end
        if ok(q) && run >= min(n_hold, numel(ok) - q + 1)
            i_resp = q;             % keeps updating to the earliest qualifying q
        end
    end
    if ~isempty(i_resp)
        R.t_resp(k)  = t(idx(i_resp)) - t_cmd(k);
        R.max_err(k) = max(abs(err(idx(i_resp):idx(end))));
        R.t_out(k)   = sum(~ok(i_resp:end))*dt;
    end
    sb = mode(idx) == 6 | mode(idx) == 7;   % STANDBY or WAKE
    sb_all(idx) = sb;
    R.standby_frac(k) = mean(sb);
    if any(sb), R.P_standby(k) = mean(p_inst(idx(sb))); end
end
R.P_standby_all = mean(p_inst(sb_all));
R.P_hold_trim   = mean(I_trim(mode == 5).^2*P.pwr.R_trim);   % trim power while tracking
R.pass_resp  = all(R.t_resp < 1e-3);
R.pass_acc   = all(R.max_err <= tol);
R.pass_power = R.P_standby_all < 1e-3;

if verbose
    fprintf('   target    response   max err after   out of spec   in standby   standby power\n');
    for k = 1:nseg
        fprintf('   %5.2f GHz  %6.2f ms   %8.2f MHz   %7.2f ms     %5.1f %%      %s\n', R.f_target(k)/1e9, ...
            R.t_resp(k)*1e3, R.max_err(k)/1e6, R.t_out(k)*1e3, 100*R.standby_frac(k), pw(R.P_standby(k)));
    end
    fprintf('   SPEC  response < 1 ms: %s (worst %.2f ms) | accuracy <= %g MHz: %s (worst %.2f MHz) | standby < 1 mW: %s (%s)\n', ...
        pf(R.pass_resp), max(R.t_resp)*1e3, tol/1e6, pf(R.pass_acc), max(R.max_err)/1e6, ...
        pf(R.pass_power), pw(R.P_standby_all));
end
end

function s = pf(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end

function s = pw(p)
if isnan(p), s = '   -   '; else, s = sprintf('%.2f mW', p*1e3); end
end
