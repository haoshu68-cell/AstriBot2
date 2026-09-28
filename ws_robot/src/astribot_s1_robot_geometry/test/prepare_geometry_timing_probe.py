#!/usr/bin/env python3
"""Make an isolated, instrumented C++ source copy without editing production.

Configure the resulting package with BUILD_TESTING=OFF and
ASTRIBOT_GEOMETRY_BUILD_PYTHON_COMPAT=OFF. The target is geometry_state_timing.
Only timestamp traces are added; no periods, stamps or validation change.
"""
import argparse
from pathlib import Path
import re
import shutil


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--compute-delay-ms', type=int, default=0,
                        help='Test-only worker delay for in-flight invalidation checks')
    args = parser.parse_args()
    if not 0 <= args.compute_delay_ms <= 1000:
        raise SystemExit('Test-only computation delay must be between 0 and 1000 ms')
    source = Path(__file__).resolve().parents[1]
    target = Path(args.output).resolve()
    if target.exists():
        raise SystemExit('Diagnostic destination must not already exist')
    shutil.copytree(source, target, ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '*.so'))
    path = target / 'src/geometry_state_node.cpp'
    text = path.read_text()
    changes = {
        'geometry_msgs::msg::Polygon rosPolygon': '''void trace(const char *event,int64_t source,int64_t ros) {
  std::cerr << "GEOMETRY_TIMING " << json{{"event",event},{"source_ns",source},{"ros_ns",ros},
    {"steady_ns",std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count()}}.dump() << std::endl;
}
geometry_msgs::msg::Polygon rosPolygon''',
        '    const auto geometry=model->geometry': '    trace("compute_begin",ns(msg.header.stamp),-1);\n    const auto geometry=model->geometry',
        '    return msg;': '    trace("compute_end",ns(msg.header.stamp),-1);\n    return msg;',
        '    const int64_t now=get_clock()->now().nanoseconds();': '    const int64_t now=get_clock()->now().nanoseconds();\n    trace("tick",0,now);',
        'msg.reason="GEOMETRY_CURRENT";publisher_->publish(msg);': 'msg.reason="GEOMETRY_CURRENT";trace("publish",ns(msg.header.stamp),now);publisher_->publish(msg);',
        '      work_context_=context();work_=': '      trace("submit",ns(msg.header.stamp),now);\n      work_context_=context();work_=',
    }
    for before, after in changes.items():
        if text.count(before) != 1:
            raise SystemExit(f'Instrumentation anchor must match exactly once: {before}')
        text = text.replace(before, after)
    if args.compute_delay_ms:
        text = text.replace('#include <future>', '#include <future>\n#include <thread>')
        text = text.replace('    const auto geometry=model->geometry',
                            '    std::this_thread::sleep_for(std::chrono::milliseconds(' +
                            str(args.compute_delay_ms) + '));\n    const auto geometry=model->geometry')
    path.write_text(text)
    cmake = target / 'CMakeLists.txt'
    cmake.write_text(re.sub(r'\bgeometry_state\b', 'geometry_state_timing', cmake.read_text()))
    print(target)


if __name__ == '__main__':
    main()
