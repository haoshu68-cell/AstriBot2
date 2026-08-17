
import importlib.util
import sys

module_name = 'load_robot'
module_path = './load_robot.so'

spec = importlib.util.spec_from_file_location(module_name, module_path)
module = importlib.util.module_from_spec(spec)
sys.modules[module_name] = module
spec.loader.exec_module(module)
