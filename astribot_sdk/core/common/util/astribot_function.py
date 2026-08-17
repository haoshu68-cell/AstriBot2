
import importlib.util
import sys

module_name = 'astribot_function'
module_path = './astribot_function.so'

spec = importlib.util.spec_from_file_location(module_name, module_path)
module = importlib.util.module_from_spec(spec)
sys.modules[module_name] = module
spec.loader.exec_module(module)
