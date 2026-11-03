function poly_out = buffer_aoi(poly_ne_m, buffer_m)
%BUFFER_AOI  Grow the AOI outward by buffer_m in every direction, so the search
%   covers the border and corners even with tracking error and corner rounding.
%   poly_ne_m 2xN [N;E] m simple polygon | buffer_m >= 0 (0 returns the input)
%   Mitered joins keep corners square, so the result contains every point within
%   buffer_m of the AOI (and a bit more at sharp corners, capped at 2*buffer_m).

    if buffer_m <= 0
        poly_out = poly_ne_m;
        return
    end
    ps = polyshape(poly_ne_m(1, :), poly_ne_m(2, :), 'Simplify', true);
    pb = polybuffer(ps, buffer_m, 'JointType', 'miter', 'MiterLimit', 2);
    if pb.NumRegions ~= 1 || pb.NumHoles ~= 0
        error('buffer_aoi:shape', 'Buffered AOI is not a single simple polygon');
    end
    poly_out = pb.Vertices.';
end
