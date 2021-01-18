from typing import NamedTuple
from functools import partial

import numpy as np
import logging
from numpy import tan, abs, array, polyfit, math
from scipy.integrate import quad
from scipy.optimize import minimize

from modules.controller.mediator import State, FormulaStateMessageType

CAR_WID = 0.75 # 0.5 of the cars wid in meters
ROAD_WID = 5


class RouteOptimizer:
    def __init__(self, state: State):
        self.p = np.zeros((4,))
        self.state = state
        self.right_bound_poly = None
        self.left_bound_poly = None

    @staticmethod
    def _is_bound_ok(p: np.ndarray, left_road_f, right_road_f, state: State, p0, p1):
        xs = np.linspace(start=0, stop=state.x_t, num=int(state.x_t / 0.5))
        is_ok = True

        for x in xs:
            r_constraint = abs(right_road_f(x) - (p[1] * x ** 3 + p[0] * x ** 2 + p1 * x + p0))
            l_constraint = abs(- left_road_f(x) + (p[1] * x ** 3 + p[0] * x ** 2 + p1 * x + p0))
            road_wid = abs(right_road_f(x) - left_road_f(x) - CAR_WID)
            is_ok = r_constraint + l_constraint <= road_wid + state.deviation
            if not is_ok:
                return -1

        return 0

    def update_optimal_route(self, state: State):

        def move_points_to(points_x, points_y, popt, move_to):
            def find_other_road_point(x, y, popt, move_to):
                a, b, c, _ = popt
                m = -1 / np.polyval([3 * a, 2 * b, c], x)
                theta = math.atan(m)
                h_x = abs(ROAD_WID * math.cos(theta))
                h_y = abs(ROAD_WID * math.sin(theta))
                if m < 0:
                    h_x *= -1
                if move_to == 'right':
                    h_x *= -1
                    h_y *= -1
                new_x = x + h_x
                new_y = y + h_y
                return new_x, new_y

            new_points_x = []
            new_points_y = []
            for x, y in zip(points_x, points_y):
                new_x, new_y = find_other_road_point(x, y, popt, move_to)
                new_points_x.append(new_x)
                new_points_y.append(new_y)
            return new_points_x, new_points_y

        self.state = state
        lx, ly = state.l_road_bound[:, 1], state.l_road_bound[:, 0]
        rx, ry = state.r_road_bound[:, 1], state.r_road_bound[:, 0]
        if len(lx) < 4:  # left list is short
            logging.info(f"Small number of left cones")
            right_road = polyfit(rx, ry, 3)
            new_x, new_y = move_points_to(rx, ry, right_road, move_to='left')
            lx += new_x
            ly += new_y
            left_road = polyfit(lx, ly, 3)
        elif len(rx) < 4:  # right list is short
            logging.info(f"Small number of right cones")
            left_road = polyfit(lx, ly, 3)
            new_x, new_y = move_points_to(lx, ly, left_road, move_to='right')
            rx += new_x
            ry += new_y
            right_road = polyfit(rx, ry, 3)
        else:  # no list is short
            left_road = polyfit(lx, ly, 3)
            right_road = polyfit(rx, ry, 3)
        self.left_bound_poly = left_road
        self.right_bound_poly = right_road

        # log printing
        logging.info(
            f"Right bound poly is: x(y) = {right_road[0]}y**3+{right_road[1]}y**2+{right_road[2]}y+{right_road[3]}")
        logging.info(
            f"Left bound poly is: x(y) = {left_road[0]}y**3+{left_road[1]}y**2+{left_road[2]}y+{left_road[3]}")

        if abs(left_road[0]) < 0.1 and abs(left_road[1]) < 0.1:
            left_road = polyfit(lx, ly, 1)
            right_road = polyfit(rx, ry, 1)
            self.left_bound_poly = [0, 0, left_road[0], left_road[1]]
            self.right_bound_poly = [0, 0, right_road[0], right_road[1]]
            # log printing
            logging.info(
                f"Right bound poly is: x(y) = {right_road[0]}y+{right_road[1]}")
            logging.info(
                f"Left bound poly is: x(y) = {left_road[0]}y+{left_road[1]}")
        return

    def get_optimal_route(self):
        return self.p


if __name__ == '__main__':
    logging.getLogger().setLevel(logging.INFO)
    state = State(r_road_bound=np.array([[1,-5.3], [2,-5], [3,-5.3], [4,-4.8], [5,-5.1],[6,-5], [7,-4.9], [8,-5.2], [9,-5], [10,-5]]),
                  l_road_bound=np.array([[1,0], [1,1], [1,2], [1,3]]),
                  pos=np.array([0, 0]),
                  angle=0,
                  x_t=10,
                  deviation=0.3,
                  dist_to_end=-1,
                  speed=1,
                  abs_pos=np.array([0, 0]),
                  abs_prev_pos=np.array([0, 0]),
                  prev_angle=0,
                  messege_type=FormulaStateMessageType.prediction_and_correction,
                  finished_lap=True)

    ro = RouteOptimizer(state)
    ro.update_optimal_route(state)
    path = ro.get_optimal_route()
    print(f'The path is: {path[0]}*x^3 + {path[1]}*x^2 + {path[2]}*x + {path[3]}')
