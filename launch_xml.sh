#!/bin/bash
MUJOCO_PATH="~/mujoco"

exec $MUJOCO_PATH/bin/simulate ./simulation/gimbal_simplified.xml
