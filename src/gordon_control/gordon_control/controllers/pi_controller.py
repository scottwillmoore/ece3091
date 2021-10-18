class PIController:
    _k_i: float
    _k_p: float

    _k_t: float

    _u_min: float
    _u_max: float

    _i: float = 0.0

    def __init__(self, *, k_p: float, k_i: float, k_t: float, u_min: float, u_max: float) -> None:
        self._k_p = k_p
        self._k_i = k_i

        self._k_t = k_t

        self._u_min = u_min
        self._u_max = u_max

    def update(self, reference: float, input: float, elapsed_time: float) -> float:
        e = reference - input

        p = self._k_p * e

        v = p + self._i
        u = min(max(self._u_min, v), self._u_max)

        e_s = u - v
        self._i += (self._k_i * e + self._k_t * e_s) * elapsed_time

        return u
