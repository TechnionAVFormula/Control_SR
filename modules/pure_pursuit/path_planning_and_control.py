import json
import sys
import os

class PathPlanningAndControl:

	_controller = None

	def __init__(self):
		try:
			config_file_path = os.path.join(os.path.dirname(__file__), 'config.json')
			with open(config_file_path, 'r') as f:
				config = json.load(f)

			module_name = config['module_name']
			car_length = config['car_length']
			kdd = config['kdd']
			max_steering_angle = config['max_steering_angle']
		except:
			print("ERROR: Something went wrong with opening or parsing the config json file!")

		self._controller = self._instance(module_name)(car_length, kdd, max_steering_angle)

	def call(self, method_name, *argv):
		return getattr(self._controller, "%s" % method_name)(*argv)

	# private methods

	def _instance(self, module_name):
		module = __import__(module_name, globals(), locals(), ['controller'], 0)
		controller_module = getattr(module, 'controller')
		class_name = self._camelize(module_name + '_controller')
		klass = getattr(controller_module, class_name)
		return klass

	def _camelize(self, class_name):
		return ''.join([word.capitalize() for word in class_name.split('_')])
