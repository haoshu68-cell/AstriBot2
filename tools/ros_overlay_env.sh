#!/usr/bin/env bash
# Rebuild only inherited ROS/colcon search entries before an explicit overlay.
# Unrelated SDK, system and user paths remain available in this child process.
astribot_prepare_overlay_environment() {
    local variable entry prefix kept skip
    local -a prefixes entries
    IFS=: read -r -a prefixes <<< "${AMENT_PREFIX_PATH-}:${COLCON_PREFIX_PATH-}"
    for variable in AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH \
            PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH PATH \
            IGN_GAZEBO_SYSTEM_PLUGIN_PATH IGN_GUI_PLUGIN_PATH IGN_GAZEBO_RESOURCE_PATH \
            GZ_SIM_SYSTEM_PLUGIN_PATH GZ_GUI_PLUGIN_PATH GZ_SIM_RESOURCE_PATH; do
        [[ -v $variable ]] || continue
        IFS=: read -r -a entries <<< "${!variable}"
        kept=""
        for entry in "${entries[@]}"; do
            [[ -n $entry ]] || continue
            skip=false
            for prefix in "${prefixes[@]}"; do
                [[ -n $prefix && $prefix != / ]] || continue
                case "$entry/" in "${prefix%/}/"*) skip=true; break;; esac
            done
            if [[ $skip == false ]]; then kept="${kept:+$kept:}$entry"; fi
        done
        if [[ -n $kept ]]; then
            printf -v "$variable" '%s' "$kept"
            export "$variable"
        else
            unset "$variable"
        fi
    done
}
