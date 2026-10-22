function [wp_ne_m, wp_type, n_wp, info] = search_boustrophedon(poly_ne_m, fov_rad, height_m, lane_heading_rad, start_ne_m) %#codegen
%SEARCH_BOUSTROPHEDON  Boustrophedon coverage path over a simple polygon (local NED).
%   poly_ne_m 2xN [N;E] m | fov_rad camera FOV across lanes | height_m AGL
%   lane_heading_rad from North toward East, NaN = auto | start_ne_m 2x1 [N;E] m
%   wp_type: 0 start, 1 transit, 2 lane, 3 connector
%   info.status: 0 ok, 1 bad input, 2 not simple, 3 buffer too small, 4 internal

    P.max_lanes         = 64;
    P.max_segs_per_lane = 8;
    P.max_wp            = 1024;
    P.geom_tol_m        = 1e-9;

    lane_spacing_m = 1.8 * height_m * tan(fov_rad / 2);

    N = size(poly_ne_m, 2);
    wp_ne_m = nan(2, P.max_wp);
    wp_type = zeros(1, P.max_wp);
    n_wp = 0;
    info = make_info(1, lane_heading_rad, 0, 0, 0, 0, 0);

    if N < 3 || ~all(isfinite(poly_ne_m(:))) || ~all(isfinite(start_ne_m(:))) ...
            || ~(fov_rad > 0 && fov_rad < pi) || ~(height_m > 0) || isinf(height_m) || isinf(lane_heading_rad) ...
            || abs(poly_area(poly_ne_m)) <= P.geom_tol_m
        return
    end
    if ~is_simple(poly_ne_m, P.geom_tol_m)
        info.status = 2;
        return
    end

    if isnan(lane_heading_rad)
        best_len = inf;
        for e = 1:N
            d_ne = poly_ne_m(:, mod(e, N) + 1) - poly_ne_m(:, e);
            if norm(d_ne) <= P.geom_tol_m
                continue
            end
            psi = mod(atan2(d_ne(2), d_ne(1)), pi);
            [w1, t1, n1, i1] = plan_one(poly_ne_m, lane_spacing_m, psi, start_ne_m, P);
            if i1.status == 0 && i1.path_len_m < best_len
                best_len = i1.path_len_m;
                wp_ne_m = w1;  wp_type = t1;  n_wp = n1;  info = i1;
            elseif isinf(best_len)
                info = i1;
            end
        end
    else
        [wp_ne_m, wp_type, n_wp, info] = plan_one(poly_ne_m, lane_spacing_m, lane_heading_rad, start_ne_m, P);
    end
end


function [wp, typ, n, info] = plan_one(poly, d, psi, p0, P)
    % Build the full coverage path for one fixed lane heading.
    N   = size(poly, 2);
    tol = P.geom_tol_m;
    ML  = P.max_lanes;
    MS  = P.max_segs_per_lane;
    MC  = MS * ML;
    wp  = nan(2, P.max_wp);
    typ = zeros(1, P.max_wp);
    n   = 0;
    info = make_info(0, psi, 0, 0, 0, 0, 0);

    u = [cos(psi); sin(psi)];
    w = [-sin(psi); cos(psi)];
    s = u' * poly;
    t = w' * poly;
    tmin = min(t);
    W = max(t) - tmin;

    nL = max(1, ceil((W - tol) / d));
    if nL > ML
        info.status = 3;
        return
    end
    tl = zeros(1, ML);
    for k = 1:nL
        tl(k) = tmin + (k - 0.5) * W / nL;
    end

    segLo = zeros(MS, ML);  segHi = zeros(MS, ML);
    eLo   = zeros(MS, ML);  eHi   = zeros(MS, ML);
    nSeg  = zeros(1, ML);
    xs = zeros(1, 2*MS);
    xe = zeros(1, 2*MS);
    for k = 1:nL
        m = 0;
        for e = 1:N
            j = mod(e, N) + 1;
            if crosses(t(e), t(j), tl(k))
                m = m + 1;
                if m > 2*MS
                    info.status = 3;
                    return
                end
                xs(m) = s(e) + (tl(k) - t(e)) / (t(j) - t(e)) * (s(j) - s(e));
                xe(m) = e;
                i = m;
                while i > 1 && xs(i-1) > xs(i)
                    tmp = xs(i); xs(i) = xs(i-1); xs(i-1) = tmp;
                    tmp = xe(i); xe(i) = xe(i-1); xe(i-1) = tmp;
                    i = i - 1;
                end
            end
        end
        if mod(m, 2) ~= 0
            info.status = 4;
            return
        end
        nSeg(k) = m / 2;
        for q = 1:nSeg(k)
            segLo(q, k) = xs(2*q - 1);  eLo(q, k) = xe(2*q - 1);
            segHi(q, k) = xs(2*q);      eHi(q, k) = xe(2*q);
        end
    end

    cellOf = zeros(MS, ML);
    nC = 0;
    for k = 1:nL
        if k > 1
            for a = 1:nSeg(k-1)
                [~, ~, endL, okL] = walk_chain(t, eLo(a, k-1), tl(k-1), tl(k));
                [~, ~, endH, okH] = walk_chain(t, eHi(a, k-1), tl(k-1), tl(k));
                if okL && okH
                    bL = find_index(eLo(:, k), nSeg(k), endL);
                    bH = find_index(eHi(:, k), nSeg(k), endH);
                    if bL > 0 && bL == bH
                        cellOf(bL, k) = cellOf(a, k-1);
                    end
                end
            end
        end
        for b = 1:nSeg(k)
            if cellOf(b, k) == 0
                nC = nC + 1;
                cellOf(b, k) = nC;
            end
        end
    end
    cellK0 = zeros(1, MC);  cellK1 = zeros(1, MC);
    for k = 1:nL
        for b = 1:nSeg(k)
            c = cellOf(b, k);
            if cellK0(c) == 0
                cellK0(c) = k;
            end
            cellK1(c) = k;
        end
    end

    [wp, typ, n, ok] = emit(wp, typ, n, p0, 0, tol);
    cur = p0;
    visited = false(1, MC);
    nOut = 0;
    for iter = 1:nC
        bestD = inf;  bc = 0;  bk = 0;  bside = 0;
        for c = 1:nC
            if visited(c)
                continue
            end
            for endIdx = 1:2
                if endIdx == 1, k = cellK0(c); else, k = cellK1(c); end
                q = find_index(cellOf(:, k), nSeg(k), c);
                for side = 1:2
                    p = lane_point(segLo(q, k), segHi(q, k), side, tl(k), u, w);
                    if norm(p - cur) < bestD
                        bestD = norm(p - cur);  bc = c;  bk = k;  bside = side;
                    end
                end
            end
        end
        visited(bc) = true;
        if bk == cellK0(bc)
            dk = 1;   kEnd = cellK1(bc);
        else
            dk = -1;  kEnd = cellK0(bc);
        end
        k = bk;
        side = bside;
        q = find_index(cellOf(:, k), nSeg(k), bc);
        pIn = lane_point(segLo(q, k), segHi(q, k), side, tl(k), u, w);
        if iter > 1 && leaves_polygon(cur, pIn, poly, tol)
            nOut = nOut + 1;
        end
        [wp, typ, n, ok1] = emit(wp, typ, n, pIn, 1, tol);  ok = ok && ok1;
        while ok
            side = 3 - side;
            [wp, typ, n, ok1] = emit(wp, typ, n, lane_point(segLo(q, k), segHi(q, k), side, tl(k), u, w), 2, tol);
            ok = ok && ok1;
            if k == kEnd
                break
            end
            if side == 1, e0 = eLo(q, k); else, e0 = eHi(q, k); end
            [nv, vidx, ~, ~] = walk_chain(t, e0, tl(k), tl(k + dk));
            for i = 1:nv
                [wp, typ, n, ok1] = emit(wp, typ, n, poly(:, vidx(i)), 3, tol);
                ok = ok && ok1;
            end
            k = k + dk;
            q = find_index(cellOf(:, k), nSeg(k), bc);
            [wp, typ, n, ok1] = emit(wp, typ, n, lane_point(segLo(q, k), segHi(q, k), side, tl(k), u, w), 3, tol);
            ok = ok && ok1;
        end
        if ~ok
            info.status = 3;
            return
        end
        cur = wp(:, n);
    end

    L = 0;
    for i = 2:n
        L = L + norm(wp(:, i) - wp(:, i-1));
    end
    info = make_info(0, psi, nL, sum(nSeg(1:nL)), nC, L, nOut);
end


function info = make_info(status, psi, nL, nS, nC, L, nOut)
    % Pack summary outputs into the info struct.
    info = struct('status', status, 'lane_heading_rad', psi, 'n_lanes', nL, ...
        'n_segments', nS, 'n_cells', nC, 'path_len_m', L, 'n_transit_outside', nOut);
end


function c = crosses(ta, tb, tline)
    % True if an edge crosses a sweep line (half-open rule, no double count at vertices).
    c = (ta <= tline && tline < tb) || (tb <= tline && tline < ta);
end


function [nv, vidx, endEdge, reached] = walk_chain(t, e0, tFrom, tTo)
    % Follow the polygon boundary from one sweep line toward the next; return vertices passed.
    N = numel(t);
    vidx = zeros(1, N);
    nv = 0;
    endEdge = e0;
    reached = false;
    fwd = (t(mod(e0, N) + 1) - t(e0)) * (tTo - tFrom) > 0;
    e = e0;
    for it = 1:N
        a = e;
        b = mod(e, N) + 1;
        if crosses(t(a), t(b), tTo)
            endEdge = e;  reached = true;
            return
        end
        if it > 1 && crosses(t(a), t(b), tFrom)
            endEdge = e;
            return
        end
        nv = nv + 1;
        if fwd
            vidx(nv) = b;
            e = mod(e, N) + 1;
        else
            vidx(nv) = a;
            e = mod(e - 2, N) + 1;
        end
    end
end


function idx = find_index(col, nUsed, val)
    % Index of val in the first nUsed entries of col (0 if absent).
    idx = 0;
    for q = 1:nUsed
        if col(q) == val
            idx = q;
            return
        end
    end
end


function p = lane_point(sLo, sHi, side, tline, u, w)
    % Convert a lane endpoint from sweep coordinates back to [N; E].
    if side == 1, sv = sLo; else, sv = sHi; end
    p = sv * u + tline * w;
end


function [wp, typ, n, ok] = emit(wp, typ, n, p, ty, tol)
    % Append a waypoint to the buffer, skipping duplicates.
    ok = true;
    if n > 0 && norm(p - wp(:, n)) <= tol
        return
    end
    if n >= size(wp, 2)
        ok = false;
        return
    end
    n = n + 1;
    wp(:, n) = p;
    typ(n) = ty;
end


function A = poly_area(poly)
    % Signed polygon area (shoelace formula).
    N = size(poly, 2);
    A = 0;
    for e = 1:N
        j = mod(e, N) + 1;
        A = A + 0.5 * (poly(1, e) * poly(2, j) - poly(1, j) * poly(2, e));
    end
end


function o = orient(a, b, c)
    % 2-D cross product: >0 if c is left of a->b.
    o = (b(1) - a(1)) * (c(2) - a(2)) - (b(2) - a(2)) * (c(1) - a(1));
    end

    function sg = sgn_tol(x, tol)
        % Sign of x with a dead zone of +/- tol.
    sg = double(x > tol) - double(x < -tol);
end


function on = on_segment(a, b, p, tol)
    % True if p lies within the bounding box of segment ab.
    on = p(1) >= min(a(1), b(1)) - tol && p(1) <= max(a(1), b(1)) + tol && ...
        p(2) >= min(a(2), b(2)) - tol && p(2) <= max(a(2), b(2)) + tol;
end


function x = segs_touch(a, b, c, d, tol)
    % True if segments ab and cd intersect or touch.
    d1 = sgn_tol(orient(c, d, a), tol);  d2 = sgn_tol(orient(c, d, b), tol);
    d3 = sgn_tol(orient(a, b, c), tol);  d4 = sgn_tol(orient(a, b, d), tol);
    x = (d1 * d2 < 0 && d3 * d4 < 0) || ...
        (d1 == 0 && on_segment(c, d, a, tol)) || (d2 == 0 && on_segment(c, d, b, tol)) || ...
        (d3 == 0 && on_segment(a, b, c, tol)) || (d4 == 0 && on_segment(a, b, d, tol));
end


function ok = is_simple(poly, tol)
    % True if no two non-adjacent polygon edges touch.
    N = size(poly, 2);
    ok = true;
    for i = 1:N
        for j = i+2:N
            if i == 1 && j == N
                continue
            end
            if segs_touch(poly(:, i), poly(:, mod(i, N) + 1), poly(:, j), poly(:, mod(j, N) + 1), tol)
                ok = false;
                return
            end
        end
    end
end


function out = leaves_polygon(a, b, poly, tol)
    % True if straight segment ab exits the polygon.
    N = size(poly, 2);
    for e = 1:N
        c = poly(:, e);
        d = poly(:, mod(e, N) + 1);
        if sgn_tol(orient(c, d, a), tol) * sgn_tol(orient(c, d, b), tol) < 0 && ...
        sgn_tol(orient(a, b, c), tol) * sgn_tol(orient(a, b, d), tol) < 0
            out = true;
            return
        end
    end
    m = 0.5 * (a + b);
    inside = false;
    for e = 1:N
        j = mod(e, N) + 1;
        if (poly(2, e) > m(2)) ~= (poly(2, j) > m(2))
            nx = poly(1, e) + (m(2) - poly(2, e)) / (poly(2, j) - poly(2, e)) * (poly(1, j) - poly(1, e));
            if m(1) < nx
                inside = ~inside;
            end
        end
    end
    out = ~inside;
end
