
import importlib.util
import sys

module_name = 'robotics_library_py'
module_path = './robotics_library_py.so'

spec = importlib.util.spec_from_file_location(module_name, module_path)
module = importlib.util.module_from_spec(spec)
sys.modules[module_name] = module
spec.loader.exec_module(module)
