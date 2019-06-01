#!/usr/bin/python

# python Slack.py  --slack_token='' 
#            --clean_channel --channel_names test --count 100
#            --list_messages --channel_names test --count 10
#            --post_delete --channel_names test
#            --channels --channel_names test  --channel_names otro --channel_names more

import os
import logging
import time
import rospy

from pprint import pprint
from optparse import OptionParser, OptionGroup
from datetime import datetime
from hs_utils import utilities 
from slackclient import SlackClient

#logging.getLogger("urllib3.connectionpool").setLevel(logging.WARNING)

class SlackHandler(object):
    
    ##TODO: Return API response when it fails
    def __init__(self, token, **kwargs):
        try:
            ## Adding local variables
            self.client         = None
            self.slack_token    = token
            self.log_level      = 'warn'
            self.detete_history = 0

            # Generating instance of strategy  
            for key, value in kwargs.iteritems():
                if "slack_token" == key:
                    self.slack_token = value

            self.client         = SlackClient(self.slack_token)
            rospy.logdebug('\t * Initialised slack client')
        except Exception as inst:
            utilities.ParseException(inst, use_ros=self.log_level)

    def PostMessage(self, channel_name, text, 
                    attachments = None, 
                    username    = '' ,
                    as_user     = True,
                    icon_emoji  = '',
                    blocks      = None
                    ):
        response = {}
        try:
            rospy.logdebug("\t * Posting new message in channel [%s]"%channel_name)
            response = self.client.api_call(
                "chat.postMessage",
                channel     = channel_name,
                text        = text,
                as_user     = as_user,
                attachments = attachments,
                username    = username,
                icon_emoji  = icon_emoji,
                blocks      = blocks #[{"type": "section", "text": {"type": "plain_text", "text": "Hello world"}}]
            )
        except Exception as inst:
            utilities.ParseException(inst, logger=self.logger)
        finally:
            return response

    ## Channel operations
    ##TODO: Make a class for this methods
    def ListChannels(self):
        response = []
        try:
            reply       =  self.client.api_call("channels.list")
            for channel in reply['channels']:
                name    = channel['name']
                id      = channel['id']
                response.append((name, id ))
                
        except Exception as inst:
            utilities.ParseException(inst, use_ros=self.log_level)
        finally:
            return response
    
    def FindChannelCode(self, channel_name):
        channel_id = None
        try:
            list_channels = self.ListChannels()
            for channel, id in list_channels:
                if channel == channel_name:
                    channel_id = id
        except Exception as inst:
            utilities.ParseException(inst, use_ros=self.log_level)
        finally:
            return channel_id
        
    def ChannelHistory(self, channel_code, count=100):
        response = None
        try:
            ## Getting channel history
            response =  self.client.api_call("channels.history", 
                                             channel=channel_code,
                                             count=count)
        except Exception as inst:
            utilities.ParseException(inst, use_ros=self.log_level)
        finally:
            return response

    def ChannelSize(self, channel_name):
        channel_size = 0
        try:
            ## Getting channel code
            channel_code    = self.FindChannelCode(channel_name)
#             if channel_code is None:
#                 rospy.logwarn("API Finding channels failed")
#                 pprint(history)
#                 return

            history         = self.ChannelHistory(channel_code)
            if 'messages' in history.keys():
                channel_size= len(history['messages'])
            
        except Exception as inst:
            utilities.ParseException(inst, use_ros=self.log_level)
        finally:
            return channel_size

    def DeleteChanngelHistory(self, channel_name):
        was_deleted      = None
        deleted_messages = 0
        channel_size     = 0
        try:
            ## Getting channel code
            channel_code = self.FindChannelCode(channel_name)
            history      = self.ChannelHistory(channel_code)
            if 'messages' in history.keys(): 
                channel_size = len(history['messages'])
            
            for message in history['messages']:
                was_deleted = self.client.api_call("chat.delete", channel=channel_code, ts=message['ts'])
                
                if not was_deleted['ok']:
                    rospy.logwarn ("%s"% was_deleted['error'])
                    return 
                deleted_messages += 1
                channel_size     -= 1
                rospy.logdebug ("Removed %d/%d message(s) [%s]"%
                                (deleted_messages, channel_size, was_deleted['ts']))

        except Exception as inst:
            utilities.ParseException(inst, use_ros=self.log_level)
        finally:
            return was_deleted, channel_size

if __name__ == '__main__':
    usage       = "usage: %prog option1=string option2=bool"
    parser      = OptionParser(usage=usage)
    parser.add_option('--slack_token',
                type="string",
                action='store',
                default=None,
                help='Input slack token')
    parser.add_option('--text',
                type="string",
                action='store',
                default="Sample text message",
                help='Input slack message')
    parser.add_option('--channel', '-c',
                type="string",
                action='store',
                default=None,
                help='Slack channel name')
    
    node_parser = OptionGroup(parser, "Retrieving options")
    node_parser.add_option('--anonymous',
                action='store_true',
                default=False,
                help='Start ROS node anonymously')
    node_parser.add_option('--debug',
                action='store_true',
                default=False,
                help='Provide debug level')
    
    cmds_parser = OptionGroup(parser, "Retrieving options")
    cmds_parser.add_option('--delete_channel_history','-d',
                action='store_true',
                default=False,
                help='Delete all messages in channel')
    cmds_parser.add_option('--list_channels',
                action='store_true',
                default=False,
                help='List of channels')
    cmds_parser.add_option('--channel_history',
                action='store_true',
                default=False,
                help='Gets history of channel baesd on channel name')
    
    parser.add_option_group(cmds_parser)
    parser.add_option_group(node_parser)
    (options, args) = parser.parse_args()
    
    ## Validating input
    if options.slack_token is None:
        parser.error("Slack token was not given, type --slack_token $SLACK_TOKEN")
    
    if options.delete_channel_history and options.channel is None:
        parser.error("Slack channel was not given, input channel (--channel)")
    
    if options.channel_history and options.channel is None:
        parser.error("Slack channel was not given, input channel (--channel)")
        
    ## Starting ros node for debugging only
    logLevel        = rospy.DEBUG if options.debug else rospy.INFO
    rospy.init_node('slack_client', anonymous=options.anonymous, log_level=logLevel)

    ## Calling slack client options
    args            = {
        "slack_token":  options.slack_token
    }
    sh = SlackHandler(options.slack_token)
    #pprint( sh.client.api_call("channels.list"))
    #print "===> token:", options.slack_token
    #pprint( sh.client.api_call("channels.info", channel="C9Y66F16D"))
    #print "---"
    #pprint( sh.client.api_call("channels.history", channel="C9Y66F16D"))
    
    #pprint( sh.client.api_call("chat.delete", channel="C9Y66F16D", ts='1557653960.002800'))
    
    if options.list_channels:
        channels = sh.ListChannels()
        pprint(channels)
    elif options.delete_channel_history:
        ok = sh.DeleteChanngelHistory(options.channel)
        if not ok:
            print "Failed to delete history"
            ## Getting channel code
    elif options.channel_history:
        channel_code = sh.FindChannelCode(options.channel)
        history      = sh.ChannelHistory(channel_code, count=1000)
        if not history['ok']:
            print "Failed to delete history"
        counter = 1
        for message in history['messages']:
            pprint(message)
            print counter," -"*50, counter
            counter += 1
