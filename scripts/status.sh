#!/bin/bash

pwd=`pwd`

package_name="mrs_uav_status"
binary="MrsUavStatus_Status"
node_name="uav_status"

this_pkg_path=$(ros2 pkg prefix $package_name)/share/$package_name

config_public=$this_pkg_path/config/public/default.yaml
platform_config=""
custom_config=""
colorscheme="COLORSCHEME_DARK"

[ -z $UAV_NAME ] && uav_name=uav1 || uav_name=$UAV_NAME
[ -z $USE_SIM_TIME ] && use_sim_time=false || use_sim_time=$USE_SIM_TIME

if [ ! -z "$PROFILES" ]; then

  is_light=$(echo "$PROFILES" | grep "COLORSCHEME_LIGHT" | wc -l)

  if [[ "$is_light" == "1" ]]; then
    colorscheme="COLORSCHEME_LIGHT"
  fi
fi

params=(
  "pwd"             "string" "$pwd"
  "colorscheme"     "string" "$colorscheme"
  "uav_name"        "string" "$uav_name"
  "enable_profiler" "bool"   "false"
  "use_sim_time"    "bool"   "$use_sim_time"
  "start_minimized" "bool"   "false"
  'config_public'   "string" "$config_public"
  'platform_config' "string" "$platform_config"
  'custom_config'   "string" "$custom_config"
)

remaps=(
  "$node_name/general_robot_info_in"       "diagnostics_manager/general_robot_info"
  "$node_name/state_estimation_info_in"    "diagnostics_manager/state_estimation_info"
  "$node_name/control_info_in"             "diagnostics_manager/control_info"
  "$node_name/collision_avoidance_info_in" "diagnostics_manager/collision_avoidance_info"
  "$node_name/uav_info_in"                 "diagnostics_manager/uav_info"
  "$node_name/system_health_info_in"       "diagnostics_manager/system_health_info"
  "$node_name/uav_state_in"                "diagnostics_manager/uav_state"
  "$node_name/display_string_in"           "display_string"
  "$node_name/goto_reference_out"          "control_manager/reference"
  "$node_name/velocity_reference_out"      "control_manager/velocity_reference"
  "$node_name/trajectory_reference_out"    "control_manager/trajectory_reference"
  "$node_name/set_constraints_out"         "constraint_manager/set_constraints"
  "$node_name/set_estimator_out"           "estimation_manager/change_estimator"
  "$node_name/set_gains_out"               "gain_manager/set_gains"
  "$node_name/set_controller_out"          "control_manager/switch_controller"
  "$node_name/set_tracker_out"             "control_manager/switch_tracker"
  "$node_name/hover_out"                   "control_manager/hover"
  "$node_name/toggle_output_out"           "control_manager/toggle_output"
  "$node_name/profiler"                    "profiler"
)

## --------------------------------------------------------------
## |                     the automatic part                     |
## --------------------------------------------------------------

CMD_BASE="ros2 run $package_name $binary"
ROS_ARGS=""

ROS_ARGS="$ROS_ARGS --remap __ns:=/$uav_name"
ROS_ARGS="$ROS_ARGS --remap __node:=$node_name"

for ((i=0; i < ${#params[*]}; i++));
do
  ((i%3==0)) && PARAM_NAME[$i/3]="${params[$i]}"
  ((i%3==1)) && PARAM_TYPE[$i/3]="${params[$i]}"
  ((i%3==2)) && PARAM_VALUE[$i/3]="${params[$i]}"
done

for ((i=0; i < ${#PARAM_NAME[*]}; i++)); do

  if [ "${PARAM_TYPE[$i]}" == "string" ]; then

    ROS_ARGS="$ROS_ARGS -p ${PARAM_NAME[$i]}:=\'${PARAM_VALUE[$i]}\'"

  elif [ "${PARAM_TYPE[$i]}" == "bool" ]; then

    ROS_ARGS="$ROS_ARGS -p ${PARAM_NAME[$i]}:=${PARAM_VALUE[$i]}"

  else

    ROS_ARGS="$ROS_ARGS -p ${PARAM_NAME[$i]}:=${PARAM_VALUE[$i]}"

  fi

done

for ((i=0; i < ${#remaps[*]}; i++));
do
  ((i%2==0)) && REMAP_FROM[$i/2]="${remaps[$i]}"
  ((i%2==1)) && REMAP_TO[$i/2]="${remaps[$i]}"
done

for ((i=0; i < ${#REMAP_FROM[*]}; i++)); do

  ROS_ARGS="$ROS_ARGS --remap ${REMAP_FROM[$i]}:=${REMAP_TO[$i]}"

done

CMD="$CMD_BASE --ros-args $ROS_ARGS"

export RCUTILS_LOGGING_USE_STDOUT=0
log_file="/tmp/mrs_uav_status_${uav_name}.log"

eval "$CMD" 2>"$log_file"
