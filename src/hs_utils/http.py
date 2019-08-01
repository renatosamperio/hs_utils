#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import json
import requests
import random
import string

from hs_utils import utilities
from pprint import pprint
from urllib import urlencode

def random_string(stringLength=10):
    """Generate a random string of fixed length """
    try:
        letters = string.ascii_lowercase
        return ''.join(random.choice(letters) for i in range(stringLength))
    except Exception as inst:
        utilities.ParseException(inst)

def post_json_call(url, json_data=None, params_data=None):
    response = None
    json_reply = None
    start_time      = time.time()
    
    if json_data is None:
        rospy.logwarn("No JSON data given for [%s]"%url)
        return
    try:
        response = requests.post(url, params = params_data, json=json_data)
        
        if response.status_code != 200:
            pprint (response.__dict__)
            rospy.logwarn("Bad request %d: %s in %s"%(response.status_code, response.reason, response.url))
        else:
            json_reply = response.json()
        
    except Exception as inst:
        utilities.ParseException(inst)
    finally:
        elapsed_time    = time.time() - start_time
        return json_reply, response, elapsed_time

def get_json_call(url, params_data=None ):
    response = None
    json_reply = None
    start_time      = time.time()
    
    try:
        if params_data is not None:
            params_data = urlencode(params_data)
        
        headers = {
            'Content-type': 'application/json', 
            'Accept': 'application/json, text/plain, */*'
        }
        response = requests.get(url, params=params_data, headers=headers)
        
        if response.status_code != 200:
            pprint (response.__dict__)
            rospy.logwarn("Bad request %d: %s in %s"%(response.status_code, response.reason, response.url))
       
    except Exception as inst:
        utilities.ParseException(inst)
    finally:
        elapsed_time    = time.time() - start_time
        return response, elapsed_time
    
def get_call(url, params_data=None):
    response = None
    json_reply = None
    start_time      = time.time()
    
    try:
        if params_data is not None:
            params_data = urlencode(params_data)
        response = requests.get(url, params=params_data)
        
        if response.status_code != 200:
            pprint (response.__dict__)
            rospy.logwarn("Bad request %d: %s in %s"%(response.status_code, response.reason, response.url))
       
    except Exception as inst:
        utilities.ParseException(inst)
    finally:
        elapsed_time    = time.time() - start_time
        return response, elapsed_time
    
def options_call(url, params_data=None):
    response = None
    json_reply = None
    start_time      = time.time()
    
    try:
        if params_data is not None:
            params_data = urlencode(params_data)
        
        headers = {
            'Content-type': 'application/json', 
            'Accept': 'application/json, text/plain, */*',
            #'Access-Control-Request-Method': 'POST\r\n',
            #'Access-Control-Request-Headers': 'content-type\r\n',
            #'Accept-Language': 'en-US,en;q=0.5\r\n',
            #'Cache-Control': 'max-age=0\r\n'
            
        }
        response = requests.options(url, params=params_data, headers=headers)
        
        if response.status_code != 200:
            pprint (response.__dict__)
            rospy.logwarn("Bad request %d: %s in %s"%(response.status_code, response.reason, response.url))
        
    except Exception as inst:
        utilities.ParseException(inst)
    finally:
        elapsed_time    = time.time() - start_time
        return response, elapsed_time