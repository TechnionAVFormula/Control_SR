from purePursuit import PurePursuitController
import math
import numpy as np
from matplotlib import pyplot

controller = PurePursuitController()
# print(controller._max_steering_angle)
# pyplot.xlim(0,700)
# pyplot.ylim(-40,600)
state = [1,0.2,1,(-math.pi/4)]
path = [1,0,0,0]

controller.update_path(path, state[2])
controller.update_state(state[0], state[1], state[2], state[3])
print(state[0], ", ", state[1], ", ", state[3])
print(controller._calculate_look_ahead_point(3))
# pyplot.arrow(state[0], state[1], 10*math.cos(state[3]), 10*math.sin(state[3]), width=1, head_width=2)
# for i in range(30):
#     state[3] = state[3] + controller.calculate_steering()/2
#     state[0] = state[0] + state[2]*math.cos(state[3])
#     state[1] = state[1] + state[2]*math.sin(state[3])
#     controller.update_state(state[0],state[1],state[2],state[3])
#     print(state[0], ", ", state[1], ", ", (state[3]*180/math.pi))
#     pyplot.arrow(state[0], state[1], 10*math.cos(state[3]), 10*math.sin(state[3]), width=1, head_width=2)

# pyplot.show()