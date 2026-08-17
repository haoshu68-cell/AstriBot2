
import importlib.util
import sys

module_name = 'whole_body_control'
module_path = './whole_body_control.so'

spec = importlib.util.spec_from_file_location(module_name, module_path)
module = importlib.util.module_from_spec(spec)
sys.modules[module_name] = module
spec.loader.exec_module(module)
