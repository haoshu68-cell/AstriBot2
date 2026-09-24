# Core line 21 failure

Root unified CTest recorded dynamics_core_differential/source boundary PASS, but arm_navigation_limit_core failed at line 21. The existing built core probe reports reach=1 and scale=0.15000000000000002: floating subtraction rounds one ULP above the binary representation of literal 0.15. Runtime algorithm matches the preserved differential oracle; the new exact-equality assertion was wrong.

Only that test assertion now accepts the closed interval [0.15,nextafter(0.15,1.0)], retaining the minimum lower bound. Source/steady 500 ms boundaries and runtime calculation are unchanged. Node clock-rollback stamp initialization now uses explicit builtin_interfaces::msg::Time() to remove the build warning. No local rebuild/ROS run; root to rerun the failed core target. Original failed logs retained under runs/mainline_20260924/nav_boundary/logs/dynamics_core_tests.log and its XML.
