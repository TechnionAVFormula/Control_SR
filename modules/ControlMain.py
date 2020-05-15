import logging
import signal

from modules.config import CONFIG
from modules.config import ConfigEnum

if CONFIG == ConfigEnum.REAL_TIME or CONFIG == ConfigEnum.COGNATA_SIMULATION:
    from pyFormulaClient import messages
elif CONFIG == ConfigEnum.LOCAL_TEST:
    from pyFormulaClientNoNvidia import messages
else:
    raise NameError('User Should Choose Configuration from config.py')

# TODO: import path is probably going to change after integration into system runner
from modules.controller.controller import BasicController
from modules.ControlClient import ControlClient

logging.basicConfig(level=logging.INFO, filename='control_BC.log')


class Control:
    def __init__(self):
        # ControlClient reads messages from modules/state_est.messages and writes to modules/control.messages
        self._client = ControlClient()
        self._running_id = 1
        self.message_timeout = 0.01
        self._controller = BasicController()
        self.num_of_calc = 0

    def start(self):
        self._client.connect(1)
        if CONFIG == ConfigEnum.LOCAL_TEST:
            self._client.set_read_delay(0.05)  # Sets the delay between reading new messages from sensors.messages
        self._client.start()

    def stop(self):
        if self._client.is_alive():
            self._client.stop()
            self._client.join()

    def process_formula_state_message(self, formula_state_msg):
        formula_state = messages.state_est.FormulaState()
        formula_state_msg.Unpack(formula_state)

        driving_instructions = messages.control.DriveInstructions()
        dash_instructions = messages.control.ControlDashbaord

        logging.info(f"**** Calculation number {self.num_of_calc} ****")
        self.num_of_calc += 1

        # Insert algorithms
        time = formula_state_msg.header.timestamp.ToMilliseconds()

        out_msg = self._controller.process_state_est(formula_state, time)
        driving_instructions.gas = out_msg.gas
        driving_instructions.breaks = out_msg.breaks
        driving_instructions.steering = out_msg.wheel_angle
        driving_instructions.speed = out_msg.speed

        # build the dashboard msg
        # dash_msg = self._controller.get_dash_msg()
        # dash_instructions.current_position = dash_msg.current_position
        # dash_instructions.current_steering_angle = dash_msg.current_steering_angle
        # dash_instructions.current_speed = dash_msg.current_speed
        # dash_instructions.gas = dash_msg.optimal_gas
        # dash_instructions.breaks = dash_msg.optimal_breaks
        # dash_instructions.optimal_speed = dash_msg.optimal_speed
        # dash_instructions.optimal_steering = dash_msg.optimal_steering
        # dash_instructions.optimalRoute = dash_msg.optimal_route
        # dash_instructions.rightBound = dash_msg.right_bound
        # dash_instructions.leftBound = dash_msg.left_bound
        # dash_instructions.rightBoundCones = dash_msg.right_bound_cones
        # dash_instructions.leftBoundCones = dash_msg.left_bound_cones

        return driving_instructions

    def process_server_message(self, server_messages):
        if server_messages.data.Is(messages.server.ExitMessage.DESCRIPTOR):
            return True

        return False

    def send_message2server(self, msg_id, driving_instruction):
        msg = messages.common.Message()
        msg.header.id = msg_id
        msg.data.Pack(driving_instruction)
        self._client.send_message(msg)

    def run(self):
        while True:
            try:
                server_msg = self._client.pop_server_message()
                if server_msg is not None:
                    if self.process_server_message(server_msg):
                        return
            except Exception as e:
                pass

            try:
                formula_state = self._client.get_formula_state_message(timeout=self.message_timeout)
                driving_instruction = self.process_formula_state_message(formula_state)
                self.send_message2server(formula_state.header.id, driving_instruction)
            except Exception as e:
                pass


control = Control()


def stop_all_threads():
    print("Stopping threads")
    control.stop()


def shutdown(a, b):
    print("Shutdown was called")
    stop_all_threads()
    exit(0)


def main():
    print("Initalized Control")
    logging.info("Initalized Control")

    control.start()
    control.run()

    stop_all_threads()
    exit(0)


if __name__ == "__main__":
    for signame in ('SIGINT', 'SIGTERM'):
        signal.signal(getattr(signal, signame), shutdown)
    main()
