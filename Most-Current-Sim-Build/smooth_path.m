function traj = smooth_path(wp_ned_m, n_wp, stop_wp, v_max_mps, a_max_mps2, psi_rad) %#codegen
%SMOOTH_PATH  Corner-rounded, speed-limited trajectory through NED waypoints.
%   wp_ned_m 3xM [N;E;D] m, first n_wp used | v_max_mps, a_max_mps2 limits | psi_rad constant yaw
%   stop_wp 1xM logical: true = come to rest at that waypoint, sharp corner (no arc).
%   Corners -> arcs of radius r = v_max^2/a_max. Corners too close for their arcs are
%   merged; a sharp corner without room for an inside arc gets a bulb turn (three
%   arcs of radius r looping outside the corner).
%   Lines: trapezoidal speed, |a| = a_max. Arcs: constant speed, v^2/r <= a_max.
%   Starts and ends at rest. Evaluate with traj_eval(traj, t_s).
%   traj.t_wp(i): time at waypoint i (arc midpoint for rounded corners).

    M = size(wp_ned_m, 2);
    MP = 4*M;
    tol = 1e-9;

    traj.n     = 0;
    traj.T     = 0;
    traj.psi   = psi_rad;
    traj.a     = a_max_mps2;
    traj.p_end = zeros(3, 1);
    traj.type  = zeros(1, MP);
    traj.t0    = zeros(1, MP);
    traj.dur   = zeros(1, MP);
    traj.p0    = zeros(3, MP);
    traj.e1    = zeros(3, MP);
    traj.e2    = zeros(3, MP);
    traj.r     = zeros(1, MP);
    traj.len   = zeros(1, MP);
    traj.th    = zeros(1, MP);
    traj.u0    = zeros(1, MP);
    traj.u1    = zeros(1, MP);
    traj.vp    = zeros(1, MP);
    traj.ta    = zeros(1, MP);
    traj.tc    = zeros(1, MP);
    traj.da    = zeros(1, MP);
    traj.dc    = zeros(1, MP);
    traj.t_wp  = zeros(1, M);

    q = zeros(3, M);
    sq = false(1, M);
    idx = ones(1, M);
    k = 0;
    for i = 1:min(n_wp, M)
        if k == 0 || norm(wp_ned_m(:, i) - q(:, k)) > tol
            k = k + 1;
            q(:, k) = wp_ned_m(:, i);
        end
        sq(k) = sq(k) || stop_wp(i);
        idx(i) = k;
    end
    if k == 0 || ~(v_max_mps > 0) || ~(a_max_mps2 > 0)
        return
    end
    traj.p_end = q(:, k);
    if k < 2
        return
    end

    r_des = v_max_mps^2 / a_max_mps2;
    r_min = r_des / 4;
    a = a_max_mps2;

    % merge adjacent corners whose arcs do not fit between them
    changed = true;
    while changed
        changed = false;
        for j = 2:k-2
            if sq(j) || sq(j+1)
                continue
            end
            d0 = unitv(q(:, j) - q(:, j-1));
            d1 = unitv(q(:, j+1) - q(:, j));
            d2 = unitv(q(:, j+2) - q(:, j+1));
            L = norm(q(:, j+1) - q(:, j));
            if L >= corner_need(turn_angle(d0, d1), r_des) + corner_need(turn_angle(d1, d2), r_des)
                continue
            end
            X = q(:, j);
            ok = false;
            c = dot(d0, d2);
            if 1 - c^2 > 1e-6
                rhs = q(:, j+1) - q(:, j);
                t1 = (dot(rhs, d0) - c*dot(rhs, d2)) / (1 - c^2);
                t2 = (dot(rhs, d2) - c*dot(rhs, d0)) / (1 - c^2);
                X = q(:, j) + t1*d0;
                ok = t1 >= 0 && t2 >= 0 && t1 <= 2*r_des && t2 <= 2*r_des && ...
                    norm(X - (q(:, j+1) - t2*d2)) <= 1e-6;
            end
            if ~ok && L < 2*r_min
                X = 0.5*(q(:, j) + q(:, j+1));
                ok = true;
            end
            if ok && turn_angle(unitv(X - q(:, j-1)), unitv(q(:, j+2) - X)) < pi - 1e-3
                q(:, j) = X;
                for m = j+1:k-1
                    q(:, m) = q(:, m+1);
                    sq(m) = sq(m+1);
                end
                for m = 1:min(n_wp, M)
                    if idx(m) > j
                        idx(m) = idx(m) - 1;
                    end
                end
                k = k - 1;
                changed = true;
                break
            end
        end
    end
    traj.p_end = q(:, k);

    dseg = zeros(3, M);
    lseg = zeros(1, M);
    for j = 1:k-1
        lseg(j) = norm(q(:, j+1) - q(:, j));
        dseg(:, j) = (q(:, j+1) - q(:, j)) / lseg(j);
    end

    % corner geometry and how much of each segment it may use
    th = zeros(1, M);
    need = zeros(1, M);
    ysd = zeros(3, M);
    for i = 2:k-1
        if ~sq(i)
            th(i) = turn_angle(dseg(:, i-1), dseg(:, i));
            ysd(:, i) = turn_side(dseg(:, i-1), dseg(:, i));
            need(i) = corner_need(th(i), r_des);
        end
    end
    availL = zeros(1, M);
    availR = zeros(1, M);
    for j = 1:k-1
        dl = 0;
        dr = 0;
        if j > 1
            dl = need(j);
        end
        if j + 1 < k
            dr = need(j+1);
        end
        L = lseg(j);
        if dl + dr <= L
            al = dl;  ar = dr;
        elseif min(dl, dr) <= L/2
            if dl <= dr
                al = dl;  ar = L - dl;
            else
                ar = dr;  al = L - dr;
            end
        else
            al = L/2;  ar = L/2;
        end
        availR(j) = al;
        availL(j+1) = ar;
    end

    % 0 none/stop, 1 inside arc, 2 bulb turn
    kind = zeros(1, M);
    Lt = zeros(1, M);
    rc = zeros(1, M);
    phi = zeros(1, M);
    cap = v_max_mps * ones(1, M);
    cap(1) = 0;
    cap(k) = 0;
    for i = 2:k-1
        if sq(i)
            cap(i) = 0;
        elseif th(i) >= 1e-6
            La = min(availL(i), availR(i));
            tn = tan(th(i)/2);
            if La >= need(i)
                kind(i) = 1;  rc(i) = r_des;  Lt(i) = need(i);
            elseif th(i) <= pi/2 || (th(i) < pi - 1e-6 && La >= r_min*tn)
                kind(i) = 1;  rc(i) = La / tn;  Lt(i) = La;
            else
                kind(i) = 2;  rc(i) = r_des;  Lt(i) = La;
                phi(i) = pi - th(i)/2 - asin(min(1, (La*cos(th(i)/2) + r_des*sin(th(i)/2)) / (2*r_des)));
            end
            if rc(i) > tol
                cap(i) = min(v_max_mps, sqrt(a * rc(i)));
            else
                kind(i) = 0;
                cap(i) = 0;
            end
        end
    end

    % pieces
    n = 0;
    w = zeros(1, MP + 1);
    cpiece = zeros(1, M);
    carc = false(1, M);
    for j = 1:k-1
        n = n + 1;
        traj.type(n) = 1;
        traj.p0(:, n) = q(:, j) + Lt(j) * dseg(:, j);
        traj.e1(:, n) = dseg(:, j);
        traj.len(n) = max(0, lseg(j) - Lt(j) - Lt(j+1));
        w(n+1) = cap(j+1);
        cpiece(j+1) = n;
        c = j + 1;
        if c < k && kind(c) == 1
            x = dseg(:, j);
            y = ysd(:, c);
            r = rc(c);
            C = q(:, c) - Lt(c)*x + r*y;
            n = n + 1;
            traj = set_arc(traj, n, C, -y, x, r, th(c));
            w(n+1) = cap(c);
            cpiece(c) = n;
            carc(c) = true;
        elseif c < k && kind(c) == 2
            x = dseg(:, j);
            y = ysd(:, c);
            r = rc(c);
            f = phi(c);
            C1 = q(:, c) - Lt(c)*x - r*y;
            n = n + 1;
            traj = set_arc(traj, n, C1, y, x, r, f);
            w(n+1) = cap(c);
            X1 = C1 + r*(cos(f)*y + sin(f)*x);
            h1 = -sin(f)*y + cos(f)*x;
            nl1 = -dot(h1, y)*x + dot(h1, x)*y;
            C2 = X1 + r*nl1;
            A = th(c) + 2*f;
            n = n + 1;
            traj = set_arc(traj, n, C2, -nl1, h1, r, A);
            w(n+1) = cap(c);
            cpiece(c) = n;
            carc(c) = true;
            X2 = C2 + r*(-cos(A)*nl1 + sin(A)*h1);
            h2 = sin(A)*nl1 + cos(A)*h1;
            nl2 = -dot(h2, y)*x + dot(h2, x)*y;
            C3 = X2 - r*nl2;
            n = n + 1;
            traj = set_arc(traj, n, C3, nl2, h2, r, f);
            w(n+1) = cap(c);
        end
    end

    % speed at piece boundaries: forward/backward pass
    for i = 1:n
        if traj.type(i) == 1
            w(i+1) = min(w(i+1), sqrt(w(i)^2 + 2*a*traj.len(i)));
        else
            w(i+1) = min(w(i+1), w(i));
        end
    end
    for i = n:-1:1
        if traj.type(i) == 1
            w(i) = min(w(i), sqrt(w(i+1)^2 + 2*a*traj.len(i)));
        else
            w(i) = min(w(i), w(i+1));
        end
    end

    t = 0;
    for i = 1:n
        u0 = w(i);
        u1 = w(i+1);
        traj.u0(i) = u0;
        traj.u1(i) = u1;
        traj.t0(i) = t;
        if traj.type(i) == 1
            L = traj.len(i);
            if L <= tol
                traj.dur(i) = 0;
                continue
            end
            vp = min(v_max_mps, sqrt(a*L + (u0^2 + u1^2)/2));
            vp = max([vp, u0, u1]);
            da = (vp^2 - u0^2) / (2*a);
            dd = (vp^2 - u1^2) / (2*a);
            dc = max(0, L - da - dd);
            traj.vp(i) = vp;
            traj.ta(i) = (vp - u0) / a;
            traj.tc(i) = dc / vp;
            traj.da(i) = da;
            traj.dc(i) = dc;
            traj.dur(i) = traj.ta(i) + traj.tc(i) + (vp - u1) / a;
        elseif traj.len(i) <= tol
            traj.dur(i) = 0;
        else
            traj.dur(i) = traj.len(i) / u0;
        end
        t = t + traj.dur(i);
    end
    traj.n = n;
    traj.T = t;

    tk = zeros(1, M);
    for c = 2:k
        pc = cpiece(c);
        if carc(c)
            tk(c) = traj.t0(pc) + traj.dur(pc)/2;
        else
            tk(c) = traj.t0(pc) + traj.dur(pc);
        end
    end
    for i = 1:min(n_wp, M)
        traj.t_wp(i) = tk(idx(i));
    end
end

% Store one arc piece: center C, unit radial e1 and tangent e2 at its start.
function traj = set_arc(traj, n, C, e1, e2, r, ang)
    traj.type(n) = 2;
    traj.p0(:, n) = C;
    traj.e1(:, n) = e1;
    traj.e2(:, n) = e2;
    traj.r(n) = r;
    traj.th(n) = ang;
    traj.len(n) = r * ang;
end

% Unit vector (zero vector stays zero).
function u = unitv(v)
    nv = norm(v);
    u = zeros(3, 1);
    if nv > 0
        u = v / nv;
    end
end

% Heading change between two unit directions [rad].
function th = turn_angle(din, dout)
    th = acos(max(-1, min(1, dot(din, dout))));
end

% Straight length an inside arc of radius r needs on each side of the corner.
function L = corner_need(th, r)
    if th < 1e-6
        L = 0;
    elseif th > pi - 1e-6
        L = inf;
    else
        L = r * tan(th/2);
    end
end

% Unit vector in the turn plane, perpendicular to din, on the side the path turns toward.
function y = turn_side(din, dout)
    y = dout - dot(dout, din)*din;
    if norm(y) < 1e-9
        y = cross([0; 0; 1], din);
        if norm(y) < 1e-9
            y = cross([1; 0; 0], din);
        end
    end
    y = y / norm(y);
end
