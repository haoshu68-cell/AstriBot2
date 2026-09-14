import inspect
import os
from datetime import datetime

class DefaultLogger:
    def _get_caller_info(self):
        """Get caller's filename and line number"""
        now = datetime.now()
        timestamp = now.strftime("%Y-%m-%d %H:%M:%S") + f".{now.microsecond // 1000:03d}"

        frame = inspect.currentframe().f_back.f_back
        filename = os.path.basename(frame.f_code.co_filename)
        line_number = frame.f_lineno
        return f"[{timestamp}][{filename}:{line_number}]"

    def info(self, message):
        prefix = self._get_caller_info()
        print(f"{prefix} [INFO] {message}")

    def debug(self, message):
        prefix = self._get_caller_info()
        print(f"{prefix} [DEBUG] {message}")

    def warning(self, message):
        prefix = self._get_caller_info()
        print(f"{prefix} [WARNING] {message}")

    def warn(self, message):
        prefix = self._get_caller_info()
        print(f"{prefix} [WARNING] {message}")

    def error(self, message):
        prefix = self._get_caller_info()
        print(f"{prefix} [ERROR] {message}")
