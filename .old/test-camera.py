from numpy import array
from numpy.linalg import inv as inverse

u, v = 462, 172

P = array(
    [
        [523.394470, 0.000000, 312.893584, 0.000000],
        [0.000000, 525.086060, 249.764264, 0.000000],
        [0.000000, 0.000000, 1.000000, 0.000000],
    ]
)

A = P[:3, :3]

fx = A[0, 0]
fy = A[1, 1]
cx = A[0, 2]
cy = A[1, 2]

x = (u - cx) / fx
y = (v - cy) / fy
z = 0.380

print(x / z)
print(y / z)

A_inverse = inverse(A)

p = array([u, v, 1])

w = A_inverse @ p
w /= w[2]
print(w)
