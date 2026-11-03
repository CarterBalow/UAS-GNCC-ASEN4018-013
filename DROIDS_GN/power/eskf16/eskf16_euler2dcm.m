function Q = eskf16_euler2dcm(e)
% 3-2-1 Euler [phi; theta; psi] -> Q_{N/B} (same matrix as QuadrotorEOM.m)
cph = cos(e(1)); sph = sin(e(1)); cth = cos(e(2)); sth = sin(e(2)); cps = cos(e(3)); sps = sin(e(3));
Q = [cth*cps, sph*sth*cps - cph*sps, cph*sth*cps + sph*sps;
     cth*sps, sph*sth*sps + cph*cps, cph*sth*sps - sph*cps;
     -sth,    sph*cth,               cph*cth];
end
