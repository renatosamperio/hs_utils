#!/usr/bin/env python

import os
import rospy
from rosgraph.roslogging import RosStreamHandler, _logging_to_rospy_names
import logging
import socket

class LoggingFormatter(object):
    def __init__(self):
        pass

    def format(self, record):
        level, color = _logging_to_rospy_names[record.levelname]
        record_message = logging._defaultFormatter.format(record)
        msg = os.environ.get('ROSCONSOLE_FORMAT', '[${severity}] [${time}]: ${message}')
        msg = msg.replace('${severity}', level)
        msg = msg.replace('${message}', str(record_message))
        msg = msg.replace('${walltime}', ''.format(rospy.get_time()))
        msg = msg.replace('${thread}', str(record.thread))
        msg = msg.replace('${logger}', str(record.name))
        msg = msg.replace('${file}', str(record.pathname))
        msg = msg.replace('${line}', str(record.lineno))
        msg = msg.replace('${function}', str(record.funcName))
        msg = msg.replace('${time}', ''.format(rospy.get_time()))
        try:
            from rospy import get_name
            node_name = get_name()
        except ImportError:
            node_name = '<unknown_node_name>'
        msg = msg.replace('${node}', node_name)
        msg += '\n'
        return msg


def remove_ros_log_rotation():
    root_handlers = logging.getLoggerClass().root.handlers
    if isinstance(root_handlers, list) and len(root_handlers)>0:
        handler = root_handlers[0]
        if type(handler) == logging.handlers.RotatingFileHandler:
            logging.getLoggerClass().root.handlers = []

def add_syslog_to_rosout(allow_std_out=False):
    rosout_logger   = logging.getLogger('rosout')

    # remove the RosStreamHandler, which prints to the console
    # to avoid duplicate entries in journalctl when nodes are started as service
    if allow_std_out and len(rosout_logger.handlers):
        for handler in rosout_logger.handlers:
            if type(handler) == RosStreamHandler:
                rospy.loginfo("Removing stream handler")
                rosout_logger.removeHandler(handler)

    ## According to RFC3164: https://tools.ietf.org/html/rfc3164
    ## Use over code 16 (local use 0-7 (LOG_LOCAL0 to LOG_LOCAL7)
    ##     import syslog
    ##     syslog.LOG_LOCAL0
    
    ## The default algorithm maps DEBUG, INFO, WARNING, ERROR 
    ## and CRITICAL to the equivalent syslog names, and all 
    ## other level names to 'warning'
    ## https://docs.python.org/3.1/library/logging.html#sysloghandler
    try:
        rospy.loginfo("Starting syslog logger")
        handler = logging.handlers.SysLogHandler(address = '/dev/log')
        handler.setFormatter(LoggingFormatter())
        rosout_logger.addHandler(handler)
    except socket.error:
        rospy.loginfo("Syslog handler not connected")

def update_loggers(allow_std_out=True):
    remove_ros_log_rotation()
    add_syslog_to_rosout(allow_std_out)
