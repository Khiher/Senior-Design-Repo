function res = yig_resonator_step(drv, mag, u, env, P)
%YIG_RESONATOR_STEP  Gap field -> YIG resonance -> filter transmission.
%   One plant step.  Codegen-compatible.
%
%   drv : [I_coil; Vc; busy; ready; I_trim]   (from yig_driver_step)
%   mag : [m; H_eff; H_coil]                  (from yig_magnet_step)
%   u   : controller command, u(6) = probe-tone frequency [Hz]
%   env : [T_degC; B_dist_T]   temperature and stray-field disturbance
%
%   res : [f0; B_gap; P_out_dBm]
%         f0        true filter centre frequency [Hz]
%         B_gap     gap flux density at the YIG sphere [T]
%         P_out_dBm power at the filter output for the probe tone [dBm]

persistent B_lag
T     = env(1);
B_dis = env(2);

B_src = P.gap.Bmax*mag(1)*(1 + P.gap.tc*(T - P.T0)) ...   % latched magnet
      + P.gap.k_main*drv(1) ...                            % main coil (pulse only)
      + P.trim.kB*drv(5) ...                               % trim coil
      + B_dis;                                             % stray field
if isempty(B_lag)
    B_lag = B_src;
end
B_lag = B_lag + P.sim.Ts/P.gap.tau_eddy*(B_src - B_lag);   % eddy-current lag

f0 = P.yig.gamma*abs(B_lag) + P.yig.f_anis + P.yig.tc_anis*(T - P.T0);

x = 2*(u(6) - f0)/P.yig.BW;
P_out = P.yig.Pin_dBm - P.yig.IL_dB - 10*log10(1 + abs(x)^(2*P.yig.nstage));
P_out = max(P_out, P.yig.floor_dBm);

res = [f0; B_lag; P_out];
end
