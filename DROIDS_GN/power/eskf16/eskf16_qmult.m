function r = eskf16_qmult(q, p)
% Hamilton product, scalar first
r = [q(1)*p(1) - q(2:4).'*p(2:4);
     q(1)*p(2:4) + p(1)*q(2:4) + cross(q(2:4), p(2:4))];
end
