function x = eskf16_predict(x, wm, am, dt, S, gN)
% One IMU step: error covariance + nominal state.
w_hat = wm - x.bg;
a_hat = am - x.ba;
Q = eskf16_q2dcm(x.q);

F = zeros(16);
F(1:3,   4:6)   =  eye(3);
F(4:6,   7:9)   = -Q*eskf16_skew(a_hat);
F(4:6,   13:15) = -Q;
F(7:9,   7:9)   = -eskf16_skew(w_hat);
F(7:9,   10:12) = -eye(3);
Fd  = F*dt;
Phi = eye(16) + Fd + 0.5*(Fd*Fd);
Qd  = diag([zeros(1,3), S.sigma_a^2*dt*ones(1,3), S.sigma_g^2*dt*ones(1,3), ...
            S.sigma_bg^2*dt*ones(1,3), S.sigma_ba^2*dt*ones(1,3), S.sigma_bb^2*dt]);
x.P = Phi*x.P*Phi.' + Qd;
x.P = 0.5*(x.P + x.P.');

q_mid = eskf16_qmult(x.q, eskf16_qexp(0.5*w_hat*dt));       % mid-step attitude
a_N   = eskf16_q2dcm(q_mid)*a_hat + gN;
x.p   = x.p + x.v*dt + 0.5*a_N*dt^2;
x.v   = x.v + a_N*dt;
x.q   = eskf16_qmult(x.q, eskf16_qexp(w_hat*dt));
x.q   = x.q/norm(x.q);
end
