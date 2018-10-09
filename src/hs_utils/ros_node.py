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

class Publisher:
    def __init__(self, **kwargs):
        try:
            self.latch          = True
            self.queue_size     = 1000
            self.message        = None
            self.message_type   = None
            self.topic          = None
            
            for key, value in kwargs.iteritems():
                if "topic" == key:
                    self.topic = value
                elif "message_type" == key:
                    self.message_type = value
                elif "queue_size" == key:
                    self.queue_size = value
                    rospy.logdebug('      Setting publisher queue size to [%s]'%str(self.queue_size))
                elif "latch" == key:
                    self.latch = value
                    rospy.logdebug('      Setting publisher message latching to [%s]'%str(self.latch))

            self.publisher  = rospy.Publisher(self.topic, 
                                              self.message_type, 
                                              queue_size=self.queue_size,
                                              latch=self.latch)

        except Exception as inst:
            ParseException(inst)
    
    def publish(self, msg):
        try:
            self.message    = msg
            if hasattr(self.message, 'header'):
                self.message.header.stamp   = rospy.Time.now()
            self.publisher.publish(msg)
        except Exception as inst:
            ParseException(inst)

    def close(self):
        self.publisher.unregister()
            
class Subscriber:
    def __init__(self, topic, messageType, lock, bag=None, execute=None):
        try:
            self.subscriber     = rospy.Subscriber(topic, messageType, self.subscriber_callback)
            self.message        = None
            self.execute        = None
            self.bag            = bag
            self.lock           = lock
            self.topic          = topic
            
            if bag is not None:
                rospy.logdebug('+   Defined ROS bag')
                self.bag        = bag

            if execute is not None:
                self.execute    = execute
        except Exception as inst:
            ParseException(inst)

    def subscriber_callback(self, data):
        '''
        Collects data and keeps message in a bag whether it exists
        '''
        try:
            with self.lock:
                self.message = data
                
                if self.execute is not None:
                    self.execute(data, self.topic)
                if self.bag is not None:
                    self.bag.write(self.topic, data)
        except Exception as inst:
            ParseException(inst)
    
    def close(self):
        with self.lock:
            self.subscriber.unregister()

class Param(object):
    '''
    TODO: Raise exception when a non setup parameter is required
    '''
    def __init__(self, **kwargs):
        try:
            self.parameter_path     = None
            self.param_value        = None
            self.param_pub          = None
            self.condition          = threading.Condition()

            for key, value in kwargs.iteritems():
                if "parameter" == key:
                    self.parameter_path = value
                    self.param_value = self.GetParam(self.parameter_path)
                    if type(self.param_value) == type({}) or type(self.param_value) == type([]):
                        rospy.logdebug('+     Updated to parameter [%s] '%(self.parameter_path))
                    else:
                        rospy.logdebug('+     Updated to parameter [%s] with [%s]'%
                                  (self.parameter_path, self.param_value))
                elif "param_pub" == key:
                    self.param_pub = value
                    rospy.logdebug('+     Updated to parameter set publisher')
            
        except Exception as inst:
              ParseException(inst)

    def GetParam(self, parameter_path):
        ''' Returns parameter value if it exists, otherwise None '''
        param_value = None
        try:
            ## Get ROS parameters
            if rospy.has_param(parameter_path):
                param_value       = rospy.get_param(parameter_path)
            else:
                rospy.logwarn('Invalid parameter path [%s] to get'%parameter_path)

        except Exception as inst:
            ParseException(inst)
        finally:
            return param_value

    def SetParam(self, parameter_path, param_value):
        '''Set ROS parameter by sending values to hive_conf module.
        '''
        try:
            ## Set ROS parameters
            if rospy.has_param(parameter_path):  
                curr_value = rospy.get_param(parameter_path)
                if isinstance(curr_value, type(1)):
                    rospy.logdebug('Setting [%s] as int for value [%s]'%(parameter_path, param_value))
                    rospy.set_param(parameter_path, int(param_value))
                elif isinstance(curr_value, type(1.0)):
                    rospy.logdebug('Setting [%s] as float for value [%s]'%(parameter_path, param_value))
                    rospy.set_param(parameter_path, float(param_value))
                elif isinstance(curr_value, type("")):
                    rospy.logdebug('Setting [%s] as string for value [%s]'%(parameter_path, param_value))
                    rospy.set_param(parameter_path, str(param_value))
                else:
                    rospy.logwarn('Setting parameter with non-defined type [%s]'%str(type(curr_value)))
                #
            else:
                rospy.logwarn('Invalid parameter path [%s] to set'%parameter_path)
        except Exception as inst:
            ParseException(inst)

class RosNode(object):
    def __init__(self, **kwargs):
        try:
            ## Adding local variables
            self.queue_size             = 1000
            self.latch                  = False
            self.lock                   = threading.Lock()
            self.mapped_subs            = {}
            self.mapped_pubs            = {}
            self.mapped_params          = {}
            self.sub_topics             = None
            self.pub_topics             = None
            self.system_params          = None
            self.bag                    = None
            self.topicConfParams        = 'smart_parameter_server'
            
            ## ROS stop signal
            rospy.on_shutdown(self.Shutdown)
            
            ## Parsing arguments
            for key, value in kwargs.iteritems():
                if "queue_size" == key:
                    self.queue_size = value
                    rospy.logdebug('+     Defined queue size of [%d] for all topics'%self.queue_size)
                elif "latch" == key:
                    self.latch = value
                    rospy.logdebug('+     Defined message latching to [%s] for all topics'%str(self.latch))
                elif "sub_topics" == key:
                    rospy.logdebug('+     Creating list of topics to subscribe')
                    self.sub_topics = value
                elif "pub_topics" == key:
                    rospy.logdebug('+     Creating list of topics to publish')
                    self.pub_topics = value
                elif "system_params" == key:
                    rospy.logdebug('+     Creating list system parameters')
                    self.system_params = value

            ## Registering subscribers and topics
            self.Register(self.sub_topics, self.pub_topics, self.system_params)
            
        except Exception as inst:
              ParseException(inst)

    def Register(self, subs, pubs, params):
        try:
            if subs is not None:
                rospy.logdebug('+ Registering subscribers and topics')
                for topic, message_type in subs:
                    rospy.logdebug('    Registering subscriber [%s]'%topic)
     
                    sub_item = {topic : Subscriber(topic, 
                                                      message_type, 
                                                      self.lock, 
                                                      execute=self.SubscribeCallback
                                                      )}
                    self.mapped_subs.update(sub_item)

            if pubs is not None:
                for topic, message_type in pubs:
                    args = {}
                    rospy.logdebug('    Registering publisher [%s]'%topic)
                    args.update({'topic':  topic})
                    args.update({'message_type':  message_type})
                    args.update({'queue_size':  self.queue_size})
                    args.update({'latch':       self.latch})
                    item_pub = {topic: Publisher(**args)}
                    self.mapped_pubs.update(item_pub)
            
            if params is not None:
                for param in params:
                    args = {}
                    args.update({'parameter':  param})
                    item_pub = {param: Param(**args)}
                    self.mapped_params.update(item_pub)
                
            ## Requires time for first connection
            if subs is not None or pubs is not None:
                time.sleep(0.5)
        except Exception as inst:
              ParseException(inst)

    def Shutdown(self):
        '''
        ROS callback method first shutdowns the publishers/subscribers and then closes the bag
        '''
        try:
            rospy.logdebug(' + Closing subscribers')
            pubKeys = self.mapped_subs.keys()
            for topic in pubKeys:
                if self.mapped_subs[topic] is not None:
                    self.mapped_subs[topic].close()
            rospy.logdebug(' +   Subscribers were unregistered')
                
            rospy.logdebug(' + Closing publishers')
            pubKeys = self.mapped_pubs.keys()
            for topic in pubKeys:
                if self.mapped_pubs[topic] is not None:
                    self.mapped_pubs[topic].close()
            rospy.logdebug(' +   Publishers were unregistered')
            
            ## TODO: Publish last state
            ## Child class can implement extra closing routines
            self.ShutdownCallback()
        except Exception as inst:
              ParseException(inst)

    def SubscribeCallback(self, msg, topic):
        raise Exception('Warning: Method SubscribeCallback() not defined in child class')

    def ShutdownCallback(self):
        ''' Shutdown method.
        Overload this method for specific shutdown routine
        '''
        try:
            rospy.logdebug('+ Shutdown: Doing nothing...')
        except Exception as inst:
              ros_node.ParseException(inst)

    def GetParam(self, param_id):
        parameter = None
        try:
            
            parameter = self.mapped_params[param_id].GetParam(param_id)   
        except Exception as inst:
              ParseException(inst)
        finally:
            return parameter

    def Publish(self, topicName, msg):
        try:
            pubTopics = self.mapped_pubs.keys()
            if topicName in pubTopics:
                self.mapped_pubs[topicName].publish(msg)
            else:
                rospy.logwarn('Topic [%s] not found in SLF ros node')
        except Exception as inst:
              ParseException(inst)

class Sample(RosNode):
    def __init__(self, **kwargs):
        try:
            
            ## Initialising parent class with all ROS stuff
            super(Sample, self).__init__(**kwargs)
            
            ## Initialise node activites
            self.Init()
        except Exception as inst:
              ParseException(inst)
              
    def SubscribeCallback(self, msg, topic):
        try:
            message = msg
            
        except Exception as inst:
              ParseException(inst)
              
    def Init(self):
        try:
            rospy.Timer(rospy.Duration(10.0), self.Run)
        except Exception as inst:
              ParseException(inst)
              
    def ShutdownCallback(self):
        try:
            rospy.logdebug('+ Shutdown: Doing nothing...')
        except Exception as inst:
              ParseException(inst)
              
    def Run(self, event):
        ''' Report Go/No every N time '''
        try:
            rospy.logdebug('+ Publishing data')
            
        except Exception as inst:
              ParseException(inst)
              
if __name__ == '__main__':
    debug_opts  = ['debug', 'warn', 'info', 'err', 'fatal']
    usage       = "usage: %prog option1=string option2=bool"
    parser      = OptionParser(usage=usage)
    parser.add_option('--queue_size',
                type="int",
                action='store',
                default=1000,
                help='Topics to play')
    parser.add_option('-l', '--latch',
                action='store_true',
                default=False,
                help='Message latching')
    parser.add_option('--run_sample',
                action='store_true',
                default=None,
                help='Message latching')
    parser.add_option('-d', '--debug',
                action='store_true',
                default=False,
                help='Provide debug level')
    parser.add_option('--do_ros_file',
                action='store_true',
                default=False,
                help='Provide debug level')
    
    (options, args) = parser.parse_args()
    
    if options.run_sample is None:
      parser.error("Missing option --run_sample")

    if options.run_sample:
        
        ## Creating ROS node
        args = {}
        logLevel       = rospy.DEBUG if options.debug else rospy.INFO
        rospy.init_node('sample', anonymous=False, log_level=logLevel)
    
        ## Removing ROS file logging
        if not options.do_ros_file:
            root_handlers = logging.getLoggerClass().root.handlers
            if isinstance(root_handlers, list) and len(root_handlers)>0:
                handler = root_handlers[0]
                if type(handler) == logging.handlers.RotatingFileHandler:
                    rospy.logwarn("Shutting down file log handler")
                    logging.getLoggerClass().root.handlers = []

        ## Defining static variables for subscribers and publishers
        sub_topics     = [
            ('topic1',  String)
        ]
        pub_topics     = [
            ('topic2',  Bool)
        ]
        system_params  = [
            '/sample_param'
        ]
        
        ## Defining arguments
        args.update({'queue_size':      options.queue_size})
        args.update({'latch':           options.latch})
        args.update({'sub_topics':      sub_topics})
        args.update({'pub_topics':      pub_topics})
        #args.update({'system_params':   system_params})
        
        # Go to class functions that do all the heavy lifting.
        try:
            spinner = Sample(**args)
        except rospy.ROSInterruptException:
            pass
        # Allow ROS to go to all callbacks.
        rospy.spin()
