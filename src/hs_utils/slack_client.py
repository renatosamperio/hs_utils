#!/usr/bin/python

# python Slack.py  --slack_token='' 
#            --clean_channel --channel_names test --count 100
#            --list_messages --channel_names test --count 10
#            --post_delete --channel_names test
#            --channels --channel_names test  --channel_names otro --channel_names more

import os
import pprint
import logging
import time
import rospy

from optparse import OptionParser, OptionGroup, OptionValueError
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

            # Generating instance of strategy  
            for key, value in kwargs.iteritems():
                if "slack_token" == key:
                    self.slack_token = value

            self.client         = SlackClient(self.slack_token)
            rospy.logdebug('+  Initialised slack client')
        except Exception as inst:
            utilities.ParseException(inst, logger=self.logger)

    def PostMessage(self, channel_name, text, 
                    attachments = None, 
                    username    = 'slack_client' ,
                    as_user     = True,
                    icon_emoji  = ':robot_face:'
                    ):
        response = {}
        try:
            rospy.logdebug("+  Posting new message in channel [%s]"%channel_name)
            response = self.client.api_call(
                "chat.postMessage",
                channel     = channel_name,
                text        = text,
                as_user     = as_user,
                attachments = attachments,
                username    = username,
                icon_emoji  = icon_emoji
            )
        except Exception as inst:
            utilities.ParseException(inst, logger=self.logger)
        finally:
            return response
