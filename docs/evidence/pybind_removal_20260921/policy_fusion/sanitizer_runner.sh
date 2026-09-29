#!/bin/sh
# Keep the host audit hook; put ASan first in the child's load order.
exec env LD_PRELOAD="/usr/lib/gcc/x86_64-linux-gnu/11/libasan.so${LD_PRELOAD:+:$LD_PRELOAD}" /tmp/codex_policy_fusion_20260921/policy_fusion_probe_sanitized "$@"
