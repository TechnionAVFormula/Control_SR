from .action_planner import ActionPlanner
from .mediator import State, OutMsg, control_state_from_est
from .route_optimizer import RouteOptimizer


class BasicController:
    def __init__(self):
        self.state: State = None
        self.route_optimizer = RouteOptimizer(state=self.state)
        self.action_planner = ActionPlanner(state=self.state)
        self.finished_lap = False
        self.first_message_time = 0

    def _update_state(self, state: State):
        for cone in state.l_road_bound:
            state.x_t = max(state.x_t, cone[0])
        state.abs_pos = state.pos
        state.abs_prev_pos = self.state.abs_pos
        state.prev_angle = self.state.angle
        converted_state = state.convert_coord_sys()
        self.state = converted_state

        if state.messege_type == finished_lap:
            self.finished_lap = True
        state.finished_lap = self.finished_lap

        if state.messege_type == prediction_and_correction:
            self.route_optimizer.update_optimal_route(self.state)
            self.action_planner.pp_controller.update_path(self.route_optimizer.get_optimal_route(), self.state.speed)
        if state.messege_type != finished_lap:
            self.action_planner.update_action(self.state, self.route_optimizer.get_optimal_route())

    def process_state_est(self, state_est, time):
        if self.first_message_time == 0:
            self.first_message_time = time
        if time - self.first_message_time < 1500: 
            out_msg = OutMsg(0,0,0,0)
            return out_msg

        state = control_state_from_est(state_est)
        self._update_state(state)

        out_msg = OutMsg(wheel_angle=self.action_planner.new_wheel_angle, speed=self.action_planner.new_speed,
                         gas=self.action_planner.new_gas, breaks=self.action_planner.new_breaks)
        return out_msg
