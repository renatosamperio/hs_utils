#!/usr/bin/env python

import sys, os
import threading
import time
import logging
import rospy

def ParseException(inst, use_ros='error'):
    ''' Takes out useful information from incoming exceptions'''
    exc_type, exc_obj, exc_tb = sys.exc_info()
    exception_fname= os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
    exception_line = str(exc_tb.tb_lineno) 
    exception_type = str(type(inst))
    exception_desc = str(inst)
    if use_ros == 'error':
        rospy.logerr("%s: %s in %s:%s"%(exception_type, 
                exception_desc, 
                exception_fname,  
                exception_line ))
    elif use_ros == 'warn':
        rospy.logwarn("%s: %s in %s:%s"%(exception_type, 
                exception_desc, 
                exception_fname,  
                exception_line ))
    else:
        print("%s: %s in %s:%s"%(exception_type, 
                exception_desc, 
                exception_fname,  
                exception_line ))

def IsNpArrayConsecutive(data, stepsize=1):
    import numpy as np
    return np.split(data, np.where(np.diff(data) != stepsize)[0]+1)

def compare_dictionaries(dict_1, dict_2, dict_1_name, dict_2_name, path="", ignore_keys=[], values=False, output=None):
    """Compare two dictionaries recursively to find non matching elements

    Args:
        dict_1: dictionary 1
        dict_2: dictionary 2

    Returns:

    """

    try:
        err = ''
        key_err = ''
        value_err = ''
        old_path = path

        ## Initialise output format    
        if output is None:
            output = {'missing': [], 'different':[]}
        
        for k in dict_1.keys():
            path = old_path + ".%s" % k
            if not dict_2.has_key(k):
                key_err += "Key %s%s not in %s\n" % (dict_1_name, path, dict_2_name)
                output['missing'].append({
                    'key': path,
                    dict_1_name: dict_1[k],
                    dict_2_name: None
                })
            else:
                if isinstance(dict_1[k], dict) and isinstance(dict_2[k], dict):
                    err_output, output = compare_dictionaries(dict_1[k],dict_2[k], dict_1_name, dict_2_name, 
                                                              path, values=values, output=output)
                    err += err_output
                elif values:
                    
                    if dict_1[k] != dict_2[k]:
                        value_err += "Value of %s%s (%s) not same as %s%s (%s)\n"\
                            % (dict_1_name, path, dict_1[k], dict_2_name, path, dict_2[k])
                        output['different'].append({
                            'key': path,
                            dict_1_name: dict_1[k],
                            dict_2_name: dict_2[k]
                        })
        
        for k in dict_2.keys():
            path = old_path + ".%s" % k
            
            ## Ignore keys that are not in the INIT message but are part of API 
            if not dict_1.has_key(k) and k not in ignore_keys:
                key_err += "Key %s%s not in %s\n" % (dict_2_name, path, dict_1_name)
                new_element = {
                    'key': path,
                    dict_2_name: dict_2[k],
                    dict_1_name: None
                }
                output['missing'].append(new_element)

        return key_err + err, output #+ value_err 
    except Exception as inst:
        sfl_utilities.ParseException(inst)