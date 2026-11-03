function cov = coverage_check(aoi_ne_m, path_ne_m, half_width_m, res_m)
%COVERAGE_CHECK  How well a flown/planned ground track covers the AOI.
%   aoi_ne_m 2xN [N;E] polygon | path_ne_m 2xK ground track (search stage)
%   half_width_m camera footprint half-width | res_m grid resolution (default 0.05 m)
%   A point counts as seen if it is within half_width_m of the track (the camera
%   footprint swept along the track, rounded at the ends: conservative).
%   cov.pct      % of the AOI seen
%   cov.margin_m worst-case margin = half_width - (largest distance from any AOI
%                point to the track). This is how much tracking error the path
%                tolerates before a gap opens; < 0 means a gap already exists.
%   cov.worst_ne_m the AOI point that is hardest to see

    if nargin < 4, res_m = 0.05; end
    lo = min(aoi_ne_m, [], 2);  hi = max(aoi_ne_m, [], 2);
    gN = lo(1):res_m:hi(1);  gE = lo(2):res_m:hi(2);
    [GN, GE] = ndgrid(gN, gE);
    in = inpolygon(GN, GE, aoi_ne_m(1, :), aoi_ne_m(2, :));

    % Resample the track to ~res_m spacing, then take the nearest track point
    % for every grid cell inside a window around it.
    seg = vecnorm(diff(path_ne_m, 1, 2));
    s = [0, cumsum(seg)];
    [s, iu] = unique(s);
    pth = interp1(s, path_ne_m(:, iu).', 0:res_m:s(end)).';
    D = inf(size(GN));
    r = half_width_m + 3*res_m;                      % window only needs to reach the footprint
    nw = ceil(r/res_m);
    for k = 1:size(pth, 2)
        i0 = round((pth(1, k) - lo(1))/res_m) + 1;  j0 = round((pth(2, k) - lo(2))/res_m) + 1;
        ii = max(1, i0 - nw):min(numel(gN), i0 + nw);
        jj = max(1, j0 - nw):min(numel(gE), j0 + nw);
        if isempty(ii) || isempty(jj), continue, end
        d = hypot(GN(ii, jj) - pth(1, k), GE(ii, jj) - pth(2, k));
        D(ii, jj) = min(D(ii, jj), d);
    end
    Din = D(in);
    cov.pct = 100*mean(Din <= half_width_m);
    [dmax, w] = max(Din);
    cov.margin_m = half_width_m - dmax;
    pts = [GN(in), GE(in)];
    cov.worst_ne_m = pts(w, :).';
end
