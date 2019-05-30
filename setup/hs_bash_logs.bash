#!/bin/bash

## This is not a executable file but a configuration
##	that is the same for all hive mind services

## Set up ROS and hive mind working space
source /opt/ros/kinetic/setup.bash

export HS_HOME=/opt/home_services/src/home_services
source $HS_HOME/devel/setup.bash

## Set up logging format as in:
## http://wiki.ros.org/rosconsole#Console_Output_Formatting
export ROSCONSOLE_FORMAT='[${severity}] [${node}:${function}] ${message}'

## Creating prioritised journal log entries
err() {
  ## This log is visible in journal 3: err level
  #echo "$SERVICE_NAME: $@" >&2
  logger -p user.error -t "$SERVICE_NAME" "$@"
}

warn() {
  ## This log is visible in journal 4: warning level
  #echo "$SERVICE_NAME: $@"
  logger -p user.warning -t $SERVICE_NAME "$@"
}

info() {
  ## This log is visible in journal 6: info level
  #echo "$SERVICE_NAME: $@"
  logger -p user.info -t $SERVICE_NAME "$@"
}

debug() {
  ## This log is visible in journal 7: debug level
  #echo "$SERVICE_NAME: $@"
  logger -p user.debug -t $SERVICE_NAME "$@"
}

if [ -z $SERVICE_NAME ]; then
	SERVICE_NAME=home_services
	info "No service name, using default: home_services"
fi

debug "Loaded base configuration"
