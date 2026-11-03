function [x, nis, ok] = eskf16_update(x, nu, H, R, gate)
% Linear ESKF update: chi^2 gate, gain, Joseph form, inject, reset.
S_  = H*x.P*H.' + R;
nis = nu.'*(S_\nu);
ok  = nis <= gate;
if ~ok, return; end
K   = (x.P*H.')/S_;
dx  = K*nu;
IKH = eye(16) - K*H;
P   = IKH*x.P*IKH.' + K*R*K.';

x.p  = x.p  + dx(1:3);
x.v  = x.v  + dx(4:6);
x.q  = eskf16_qmult(x.q, eskf16_qexp(dx(7:9)));  x.q = x.q/norm(x.q);
x.bg = x.bg + dx(10:12);
x.ba = x.ba + dx(13:15);
x.bb = x.bb + dx(16);

G = eye(16);  G(7:9,7:9) = eye(3) - 0.5*eskf16_skew(dx(7:9));
x.P = G*P*G.';
x.P = 0.5*(x.P + x.P.');
end
