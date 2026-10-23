function [p_ned_m, v_ned_mps, a_ned_mps2, psi_rad, psi_dot_rps] = traj_eval(traj, t_s) %#codegen
%TRAJ_EVAL  Reference p, v, a (NED, SI) and yaw at time t_s on a smooth_path trajectory.
%   Before t = 0 holds the start; after traj.T holds the final point at rest.

    psi_rad = traj.psi;
    psi_dot_rps = 0;
    v_ned_mps = zeros(3, 1);
    a_ned_mps2 = zeros(3, 1);
    p_ned_m = traj.p_end;
    if traj.n == 0
        return
    end

    t = min(max(t_s, 0), traj.T);
    lo = 1;
    hi = traj.n;
    while lo < hi
        mid = floor((lo + hi + 1) / 2);
        if traj.t0(mid) <= t
            lo = mid;
        else
            hi = mid - 1;
        end
    end
    i = lo;
    tau = min(t - traj.t0(i), traj.dur(i));
    a = traj.a;

    if traj.type(i) == 1
        u0 = traj.u0(i);
        vp = traj.vp(i);
        ta = traj.ta(i);
        tc = traj.tc(i);
        if traj.dur(i) == 0
            s = 0;  sp = u0;  acc = 0;
        elseif tau < ta
            s = u0*tau + 0.5*a*tau^2;  sp = u0 + a*tau;  acc = a;
        elseif tau < ta + tc
            s = traj.da(i) + vp*(tau - ta);  sp = vp;  acc = 0;
        else
            t3 = tau - ta - tc;
            s = traj.da(i) + traj.dc(i) + vp*t3 - 0.5*a*t3^2;  sp = vp - a*t3;  acc = -a;
        end
        p_ned_m = traj.p0(:, i) + s * traj.e1(:, i);
        v_ned_mps = sp * traj.e1(:, i);
        a_ned_mps2 = acc * traj.e1(:, i);
    else
        u = traj.u0(i);
        r = traj.r(i);
        al = u * tau / r;
        radial = cos(al) * traj.e1(:, i) + sin(al) * traj.e2(:, i);
        tangent = -sin(al) * traj.e1(:, i) + cos(al) * traj.e2(:, i);
        p_ned_m = traj.p0(:, i) + r * radial;
        v_ned_mps = u * tangent;
        a_ned_mps2 = -(u^2 / r) * radial;
    end
end
