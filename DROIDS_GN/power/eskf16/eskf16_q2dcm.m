function Q = eskf16_q2dcm(q)
% q -> Q_{N/B} (body -> NED)
qw = q(1); qv = q(2:4);
Q = (qw^2 - qv.'*qv)*eye(3) + 2*(qv*qv.') + 2*qw*eskf16_skew(qv);
end
