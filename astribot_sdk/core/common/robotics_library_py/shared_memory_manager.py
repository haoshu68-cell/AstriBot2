
import importlib.util
import sys

module_name = 'shared_memory_manager'
module_path = './shared_memory_manager.so'

spec = importlib.util.spec_from_file_location(module_name, module_path)
module = importlib.util.module_from_spec(spec)
sys.modules[module_name] = module
spec.loader.exec_module(module)
