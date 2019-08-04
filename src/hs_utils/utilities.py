#!/usr/bin/env python

import sys, os
import threading
import time
import logging
import rospy
import unicodedata

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

def compare_dictionaries(dict_1, dict_2, dict_1_name, dict_2_name, path="", ignore_keys=[], use_values=False, output=None):
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
            path = old_path + ".%s" % k if len(old_path)>0 else "%s" % k

#             print "===> variable:", k
#             print "===>",k," not in ignore_keys:", k not in ignore_keys
            
            if k not in ignore_keys:
#                 print "  ===> dict_2.has_key(",k,"):", dict_2.has_key(k)
                if dict_2.has_key(k):
#                     print "    ===> key:", k
#                     print "    ===> dict_1[",k,"]:", dict_1[k]
#                     print "    ===> dict_2[",k,"]:", dict_2[k]
#                     print ""
                    ## key do not exi
                    if isinstance(dict_1[k], dict) and isinstance(dict_2[k], dict):
                        err_output, output = compare_dictionaries(dict_1[k],dict_2[k], dict_1_name, dict_2_name, 
                                                                  path, use_values=use_values, output=output)
                        err += err_output
                    elif use_values:
                        if k not in ignore_keys:
    #                         print "  ===> dict_1["+k+"]:", dict_1[k]
    #                         print "  ===> dict_2["+k+"]:", dict_2[k]
    #                         print "  ===> Equal?", dict_1[k] != dict_2[k]
                            if dict_1[k] != dict_2[k]:
                                value_err += "Value of %s.%s (%s) not same as %s.%s (%s)\n"\
                                    % (dict_1_name, path, dict_1[k], dict_2_name, path, dict_2[k])
                                err += value_err
                                output['different'].append({
                                    'key': path,
                                    dict_1_name: dict_1[k],
                                    dict_2_name: dict_2[k]
                                })
                    
                #if not dict_2.has_key(k) and k not in ignore_keys:
                else:
                    key_err += "(1) Key %s not in %s\n" % (path, dict_2_name)
                    output['missing'].append({
                        'key': path,
                        dict_1_name: dict_1[k],
                        dict_2_name: None
                    })
        
        for k in dict_2.keys():
            path = old_path + ".%s" % k if len(old_path)>0 else "%s" % k
            
            ## Ignore keys that are not in the INIT message but are part of API 
            if not dict_1.has_key(k) and k not in ignore_keys:
                key_err += "(2) Key %s not in %s\n" % (dict_2_name, path, dict_1_name)
                new_element = {
                    'key': path,
                    dict_2_name: dict_2[k],
                    dict_1_name: None
                }
                output['missing'].append(new_element)

    except Exception as inst:
        ParseException(inst)
    finally:
        return key_err + err, output #+ value_err 

def convert_to_str(item):
    try:
        if isinstance(item, list):
            for i in range(len(item)):
                #print "  ---> E: ", item[i], "::", type(item[i])
                if isinstance(item[i], unicode):
                    item[i] = unicodedata.normalize('NFKD', item[i]).encode('ascii','ignore')
                    #print "  ---> E.R: ", item[i], "::", type(item[i])
                elif isinstance(item[i], dict):
                    item[i] = convert_to_str(item[i])
        elif isinstance(item, dict):
            for key in item.keys():
                #print "  ---> D[",key,"]: ", item[key], "::", type(item[key])
                if isinstance(item[key], unicode):
                    item[key] = unicodedata.normalize('NFKD', item[key]).encode('ascii','ignore')
                    #print "  ---> D2[",key,"]: ", item[key], "::", type(item[key])
                elif isinstance(item[key], dict):
                    item[key] = convert_to_str(item[key])
                elif isinstance(item[key], list):
                    #print "---> key:", key
                    item[key] = convert_to_str(item[key])
            
    except Exception as inst:
        ParseException(inst)
    finally:
        return item
