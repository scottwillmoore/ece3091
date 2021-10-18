class PIDController:
    k_p: float
    k_i: float
    k_d: float

    k_t: float

    _u_min: float
    _u_max: float

    _i: float = 0.0
    _d: float = 0.0

    def __init__(self, k_p, k_i, k_d, *, k_t: float = 0.0, u_min: float, u_max: float) -> None:
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        self._k_t = k_t

        self._u_min = u_min
        self._u_max = u_max

    def update(self, reference: float, input: float, elapsed_time: float) -> float:
        e = reference - input

        p = self.k_p * e
        self._d = 0.0

        v = p + self._i + self._d
        u = min(max(self._u_min, v), self._u_max)

        e_s = u - v
        self._i += (self.k_i * e + self._k_t * e_s) * elapsed_time

        return u

