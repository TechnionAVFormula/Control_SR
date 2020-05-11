from typing import NamedTuple
from functools import partial

import numpy as np
from numpy import tan, abs, array, polyfit
from scipy.integrate import quad
from scipy.optimize import minimize

from .mediator import State


class RouteOptimizer:
    def __init__(self, state: State):
        self.p = None
        self.state = state

    @staticmethod
    def _is_bound_ok(p: np.ndarray, left_road_f, right_road_f, state: State, p0, p1):
        xs = np.linspace(start=0, stop=state.x_t, num=int(state.x_t / 0.5))
        is_ok = True

        for x in xs:
            r_constraint = abs(right_road_f(x) - (p[1] * x ** 3 + p[0] * x ** 2 + p1 * x + p0))
            l_constraint = abs(- left_road_f(x) + (p[1] * x ** 3 + p[0] * x ** 2 + p1 * x + p0))
            road_wid = abs(right_road_f(x) - left_road_f(x))
            is_ok = r_constraint + l_constraint <= road_wid + state.deviation
            if not is_ok:
                return -1

        return 0

    def update_optimal_route(self, state: State):
        self.state = state
        p0 = state.pos[1]
        p1 = tan(state.angle)

        # Calculate polynomial-fit coefficients of the left, right edges
        left_road = polyfit(state.l_road_bound[:, 0], state.l_road_bound[:, 1], 3)
        right_road = polyfit(state.r_road_bound[:, 0], state.r_road_bound[:, 1], 3)

        # if the road is straight - no need for complex optimization
        if abs(left_road[0]) < 0.1 and abs(left_road[1]) < 0.1:
            left_road = polyfit(state.l_road_bound[:, 0], state.l_road_bound[:, 1], 1)
            right_road = polyfit(state.r_road_bound[:, 0], state.r_road_bound[:, 1], 1)
            self.p = [0, 0, (left_road[0]+right_road[0])/2,  (left_road[1]+right_road[1])/2]
            return

        # otherwise, optimize intensely!
        road = lambda x, p2, p3: p3 * x ** 3 + p2 * x ** 2 + p1 * x + p0
        road_dx = lambda x, p2, p3: 3 * p3 * x ** 2 + 2 * p2 * x + p1
        road_d2x = lambda x, p2, p3: 6 * p3 * x + 2 * p2

        left_road_f = lambda x: left_road[0] * x ** 3 + left_road[1] * x ** 2 + left_road[2] * x + left_road[3]
        right_road_f = lambda x: right_road[0] * x ** 3 + right_road[1] * x ** 2 + right_road[2] * x + right_road[3]

        # radius of curvature
        integrand_rc = lambda x, p2, p3: abs(1 + road_dx(x, p2, p3) ** 2) ** (3 / 2) / abs(road_d2x(x, p2, p3))
        partial_int_rc = lambda p2, p3: quad(integrand_rc, a=0, b=state.x_t, args=(p2, p3))

        # middle distance
        integrand_md = lambda x, p2, p3: (0.5 * (right_road_f(x) - left_road_f(x)) - road(x, p2, p3)) ** 2
        partial_int_md = lambda p2, p3: quad(integrand_md, a=0, b=state.x_t, args=(p2, p3))

        # optimization target
        if state.finished_lap:
            target = lambda p: partial_int_rc(p[0], p[1])[0]
        else:
            target = lambda p: partial_int_rc(p[0], p[1])[0] + partial_int_md(p[0], p[1])[0]

        p_min = minimize(
            target, x0=array([1, 1]),
            constraints=({'type': 'ineq', 'fun': partial(self._is_bound_ok,
                                                         left_road_f=left_road_f, right_road_f=right_road_f,
                                                         state=state, p0=p0, p1=p1)}))
        self.p = np.array([p_min[1], p_min[0], p1, p0])

    def get_optimal_route(self):
        return self.p


if __name__ == '__main__':
    state = State(r_road_bound=np.array([[1,0], [2,0.1], [3,-0.3], [4,0], [5,0.3], [6,0.2], [7,0], [8,0.1], [9,-0.3], [10,0.3]]),
                  l_road_bound=np.array([[1,-5.3], [2,-5], [3,-5.3], [4,-4.8], [5,-5.1],[6,-5], [7,-4.9], [8,-5.2], [9,-5], [10,-5]]),
                  pos=np.array([0, 0]),
                  angle=0,
                  x_t=10,
                  deviation=0.3,
                  is_course_complete=False,
                  dist_to_end=-1,
                  speed=1,
                  abs_pos=np.array([0, 0]),
                  abs_prev_pos=np.array([0, 0]),
                  prev_angle=0)

    ro = RouteOptimizer()
    ro.update_optimal_route(state)
    path = ro.get_optimal_route()
    print(f'The path is: {path[0]}*x^3 + {path[1]}*x^2 + {path[2]}*x + {path[3]}')
