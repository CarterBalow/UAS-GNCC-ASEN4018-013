function q = eskf16_qexp(phi)
% rotation vector -> unit quaternion
n = norm(phi);
if n < 1e-8
    q = [1; 0.5*phi];  q = q/norm(q);
else
    q = [cos(n/2); (phi/n)*sin(n/2)];
end
end
