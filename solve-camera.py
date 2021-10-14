from numpy import array
from cv2 import SOLVEPNP_ITERATIVE, solvePnP as solve_pnp

# Better image, with more variety of points...
# Maybe attempt an extrinsic guess...

object_points = array(
    [
        (-0.1, 0.3, 0.0),
        (0.0, 0.3, 0.0),
        (0.1, 0.4, 0.0),
        (-0.1, 0.4, 0.0),
        (0.0, 0.4, 0.0),
        (0.1, 0.4, 0.0),
        (-0.2, 0.5, 0.0),
        (-0.1, 0.5, 0.0),
        (0.0, 0.5, 0.0),
        (0.2, 0.6, 0.0),
        (0.1, 0.5, 0.019),
    ],
    dtype="double",
)

image_points = array(
    [
        (81, 406),
        (328, 405),
        (582, 403),
        (149, 266),
        (325, 263),
        (504, 261),
        (51, 192),
        (187, 189),
        (324, 187),
        (547, 134),
        (464, 160),
    ],
    dtype="double",
)

fx = 523.394470
fy = 525.086060

cx = 312.893584
cy = 249.764264

camera_matrix = array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype="double")

distortion_coefficients = array([0.166029, -0.258564, 0.000484, -0.003085, 0], dtype="double")

success, rotation, translation = solve_pnp(
    object_points,
    image_points,
    camera_matrix,
    distortion_coefficients,
    useExtrinsicGuess=False,
    flags=SOLVEPNP_ITERATIVE,
)

print(success)
print(rotation)
print(translation)
