#!/usr/bin/env python

import rospy
import sys, os
import threading
import time

from optparse import OptionParser
from std_msgs.msg import Bool
from std_msgs.msg import String

def ParseException(inst):
  ''' Takes out useful information from incoming exceptions'''
  exc_type, exc_obj, exc_tb = sys.exc_info()
  exception_fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
  exception_line = str(exc_tb.tb_lineno) 
  exception_type = str(type(inst))
  exception_desc = str(inst)
  rospy.logerr( "  %s: %s in %s:%s"%(exception_type, 
                exception_desc, 
                exception_fname,  
                exception_line ))

def IsNpArrayConsecutive(data, stepsize=1):
    import numpy as np
    return np.split(data, np.where(np.diff(data) != stepsize)[0]+1)
