#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import sys, os
import rospy
import logging
import datetime
import re
import json

import numpy as np
from pprint import pprint
from hs_utils import utilities 
from hs_utils import similarity
from hs_utils.mongo_handler import MongoAccess
from optparse import OptionParser, OptionGroup
from operator import itemgetter
from imdbpie import Imdb

class IMDbHandler:
    def __init__(self, **kwargs):
        ''' '''
        try: 
            ## Adding local variables
            self.clean_chars = '[-«\[@#$()\]]'
            self.non_valid  = ['TV episode', 'video game', 'TV short', 'TV series', 'TV mini-series', 'video', 'TV special']
            self.valid = ['feature', 'TV movie']
            self.list_terms = {'case_sensitive': [], 'lower_case': [], 'depends_context': []}
            self.comparator = similarity.Similarity()

            # Generating instance of strategy  
            for key, value in kwargs.iteritems():
                if "list_terms" == key:
                    if value is not None:
                        rospy.logdebug("  +   Loading list of terms")
                        for file in os.listdir(value):
                            if file.endswith(".txt"):
                                file_path = os.path.join(value, file)
                                self.list_terms.update({file[:-4]: self.load_terms(file_path)})

            ## Converting list of termins into UTF-8
            self.case_sensitive  = [item.decode('utf8') for item in self.list_terms['case_sensitive']]
            self.lower_case      = [item.decode('utf8') for item in self.list_terms['lower_case']]
            self.depends_context = [item.decode('utf8') for item in self.list_terms['depends_context']]

        except Exception as inst:
            utilities.ParseException(inst)

    def load_terms(self, fileName):
        try:
            with open(fileName, 'r') as file:
                return file.read().strip().split()
                
        except Exception as inst:
            utilities.ParseException(inst)

    def list_values(self, new_item, key):
        try:
            key_item    = new_item[key]['value']
            item_month  = key_item.keys()[0]
            item_days   = key_item[item_month].keys()
            lst_values  = [key_item[item_month][day] for day in item_days]
            lst_values  = list(map(int, lst_values))
            array_values=np.array(lst_values)
            return array_values
        except Exception as inst:
          utilities.ParseException(inst)
    
    def clean_sentence(self, sentence, debug=False):
        try:
            new_sentence    = []
            if self.list_terms is None:
                rospy.logwarn("Invalid list of terms")
                return
            
            ## Removing non-expected characters
            sentence = re.sub('[\[@#$()\]]', '', sentence)

            ## Removing special works from torrent
            splitted        = sentence.strip().split()
            if debug: print "---> 1sentence:\t\t ", sentence 

            ## Split sentence and look if every word
            ##    is in the special list of characters
            ##    to remove.
            started_bad_section = 0
            for token in splitted:
                
                if debug: print "     1token:\t ", token
                ## Discarding reserved words for torrent
                ##    typical descriptions
                if token in self.list_terms['case_sensitive']:
                    started_bad_section += 1
                    continue
                if debug: print "       11token:\t ", token
                
                ## Discarding generic lower case
                ##    list of terms
                if token.lower() in self.list_terms['lower_case']:
                    started_bad_section += 1
                    continue
                if debug: print "       12token:\t ", token
                
                ## If sentence already shows bad words
                ##    consider extra words that normally
                ##    would be normal words
                if started_bad_section>1 and token in self.list_terms['depends_context']:
                    started_bad_section += 1
                    continue
                if debug: print "       13token:\t ", token

                new_sentence.append(token)
                if debug: print "     2token:\t ", token

            new_sentence    = ' '.join(new_sentence)
            if debug: print "---> 2new_sentence:\t ", new_sentence
            
            ## Removing non-expected characters
            new_sentence = re.sub('[\[@#$()\]]', '', new_sentence)
            if debug: print "---> 3new_sentence:\t ", new_sentence
            
#             RX = re.compile('([&#])')
#             new_sentence = RX.sub(r'\\\1', new_sentence)

        except Exception as inst:
          utilities.ParseException(inst)
        finally:            
            return new_sentence

    def skim_title(self, torrent_title):
        splitted = None
        year_found = None
        try:
#             
            ## Using torrent title until year,
            ##   only if torrent title has a year
            title_has_year  = re.match(r'.*([12].[0-9]{2})', torrent_title)
            #title_has_year  = re.match(r'\d\d\d\d', torrent_title)
            
            if title_has_year is not None:
                year_found  = title_has_year.group(1)
                
                ## Adding torrent title year
                splitted    = torrent_title.split(year_found)[0]
            else:
                splitted    = torrent_title
                
            ## Remove non-required specific characters
            if not splitted.isalpha():
                splitted = u''.join(splitted).encode('utf-8').strip()
                splitted = splitted.translate(None, b"-([_#@&*|~`%^<>]").strip()

#             print "  ===> title_has_year:\t ", title_has_year
#             print "  ===> year_found:\t ", year_found
#             print "  ===> splitted:\t ", splitted
        except Exception as inst:
          utilities.ParseException(inst)
        finally:
            return year_found, splitted

    def get_titles(self, imdb, title):
        listed_items = []
        try:
            try:
                imdb_data       = imdb.search_for_title(title)
            except ValueError:
                rospy.logwarn('IMDb did not find titles for [%s]'%title)
                return
            
            for item in imdb_data:
                if item['type'] in self.non_valid:
                    continue
                elif item['type'] not in self.valid:
                    unknow_type = json.dumps(item, sort_keys=True, indent=4, separators=(',', ': '))
                    rospy.loginfo('Non-listed item:\n%s'% unknow_type)
                listed_items.append(item)

        except Exception as inst:
          utilities.ParseException(inst)
        finally:
            return listed_items
        
    def get_imdb_best_title(self, splitted_title, year_found=None):
        try:
            imdb            = Imdb()
            updated_imdb    = []
            imdb_selected   = []
            item_selected   = {}

            if len(splitted_title)<1:
                return
            
            ## Getting IMDB information
            imdb_data       = self.get_titles(imdb, splitted_title)
            total_items     = len(imdb_data)
            rospy.logdebug("+ Getting IMDB [%s] items"%(str(total_items)))
            if total_items < 0:
                rospy.logwarn("No titles were found for [%s]"%splitted_title)
                return
            
            ## Pre-selecting only IMDB titles that 
            ##   look very similar to torrent title
            ignored_items   = 0
            for imdb_item in imdb_data:
                debug = False
#                 if 'The Call'==splitted_title:
#                     debug = True
                score = self.comparator.score(splitted_title, imdb_item['title'], debug=debug)
                year_matches    = year_found == imdb_item['year']
                item_type       = 'feature' == imdb_item['type']
                imdb_item.update({'score':score})
                imdb_item.update({'year_matches':year_matches})
                imdb_item['title'] = u''.join(imdb_item['title']).encode('utf-8').strip()
                
                splitted_title_decoded  = splitted_title.decode('utf-8')
                ## Adding only very similar titles
                if score > 0.98:
                    updated_imdb.append(imdb_item)
                else:
                    ignored_items           += 1
                    imdb_title_decoded      = imdb_item['title'].decode('utf-8')
                    
                    rospy.logdebug("      Ignored [%s] vs [%s] = %f"%
                                      (splitted_title_decoded, imdb_title_decoded, score))
                    
            rospy.logdebug("+   Ignored [%s] item(s)"%(str(ignored_items)))
            
            ## If all items are ignored take the highest one
            if len(imdb_data)<1:
                return
                
            if total_items == ignored_items:
                sorted_imdb = sorted(imdb_data, key=itemgetter('score'), reverse=True) 
                rospy.logwarn("Taking highest score for [%s]"%splitted_title_decoded)
                imdb_item       = sorted_imdb[0]
                updated_imdb.append(imdb_item)
            
            ## Sorting IMDB retrieved items by similarity score
            sorted_imdb = sorted(updated_imdb, key=itemgetter('score'), reverse=True) 
            
            ## Checking if torrent year matches, otherwise 
            ##   provide only feature type IMDB items
            better_item_not_found           = False
            for imdb_item in sorted_imdb:
                item_added                  = False
                new_item                    = {}
                if imdb_item['year_matches']:
                    better_item_not_found   = True
                    item_added              = True
                elif not better_item_not_found and 'feature' == imdb_item['type']:
                    item_added              = True
                
                # Discard non-movie items
                if imdb_item['type'] in self.non_valid:
                    rospy.logwarn("+   Discarding [%s] of type [%s]"%
                                  (imdb_item['title'], imdb_item['type']))
                    break
                
                ## Retrieving additional IMDB information
                ##   and adding item
                if item_added:
                    imdb_id                 = str(imdb_item['imdb_id'])
                    try:
                        title_genres        = imdb.get_title_genres(imdb_id)
                        
                        ## Finding genre 
                        if title_genres is not None and 'genres' in title_genres:
                            genres_label         = str(', '.join(title_genres['genres']))
                            imdb_item.update({'genres': genres_label})

                    except LookupError as imdb_error:
                        rospy.logdebug("+   Title genres not found for IMDB id [%s]"%imdb_id)
                        utilities.ParseException(imdb_error)
                    
                    try:
                        title_info          = imdb.get_title(imdb_id)
                        
                        ## Finding image
                        imdb_image_url          = ''
                        if 'image' in title_info['base'].keys():
                            imdb_image_url          = title_info['base']['image']['url']
                        else:
                            rospy.logdebug("-   Image URL not found")
                            rospy.logdebug("-   Looking in similarities for images...")
                            if 'similarities' not in title_info.keys():
                                rospy.logdebug("-   Similarities sections not found")
                                
                            else:
                                for similarity_item in title_info['similarities']:
                                    itemKeys        = similarity_item.keys()
                                    if 'image' in itemKeys:
                                        imageKey    = similarity_item['image'].keys()
                                        if 'url' in imageKey:
                                            imdb_image_url = similarity_item['image']['url']
                                            rospy.logdebug("      Using image from similarities")
                                            break
                        ## Finding raiting
                        imdb_raiting            = ''
                        if 'rating' in title_info['ratings'].keys():
                            imdb_raiting        = str(title_info['ratings']['rating'])
                        else:
                            rospy.logdebug("-   Raiting not found")
                        
                        ## Finding movie plot
                        imdb_plot               = ''
                        if 'plot' in title_info.keys() and 'outline' in title_info['plot'].keys():
                            imdb_plot           = title_info['plot']['outline']['text']
                        else:
                            rospy.logdebug("-   Plot not found")
    
                        ## Creating data structure
                        imdb_title_url          = title_info['base']['id']
                        imdb_item.update({'raiting':    imdb_raiting})
                        imdb_item.update({'plot':       imdb_plot})
                        imdb_item.update({'image_url':  imdb_image_url})
                        imdb_item.update({'title_url':  'http://www.imdb.com/'+imdb_title_url})
                        
                        if year_found is not None:
                            imdb_item.update({'year': year_found})
                        imdb_selected.append(imdb_item)
                    except LookupError as imdb_error:
                        rospy.loginfo("+   Title info not found for IMDB id [%s]"%imdb_id)
                        utilities.ParseException(imdb_error)

            item_selected.update({'imdb_info':imdb_selected})
            item_selected.update({'query_title':splitted_title})
        except Exception as inst:
          utilities.ParseException(inst)
        finally:
            return item_selected

def test_with_db(args):
    try:
        # Go to class functions that do all the heavy lifting.
        try:
            database        = 'limetorrents'
            collection      = 'movies'
            db_handler      = MongoAccess()
            db_handler.Connect(database, collection)
            records         = db_handler.Find()
            rospy.logdebug("  + Found [%d] records"%records.count())
            
            counter         = 1
            movie_handler   = IMDbHandler(**args)
            rospy.logdebug("  + Created IMDb handler")
            
            for record in records:
                if counter > 5:
                    break
                
                title       = record['name']
                clean_name  = movie_handler.clean_sentence(title)
                rospy.logdebug("  + [%d] Skimmed title: [%s]"%(counter, clean_name))
                items       = movie_handler.get_imdb_best_title(clean_name)
                
                pprint(items)
                print "-"*80
                counter += 1
            # Allow ROS to go to all callbacks.
            #rospy.spin()
        except rospy.ROSInterruptException:
            pass
    except Exception as inst:
        utilities.ParseException(inst)

if __name__ == '__main__':
    logging.getLogger('imdbpie').setLevel(logging.getLevelName('DEBUG'))
    usage       = "usage: %prog option1=string option2=bool"
    parser      = OptionParser(usage=usage)
    parser.add_option('--debug',
                action='store_true',
                default=False,
                help='Provide debug level')
    parser.add_option('--list_terms',
                type="string",
                action='store',
                default=None,
                help='Input list of term path')
    
    (options, args) = parser.parse_args()
    
    if options.list_terms is None:
       parser.error("Missing required option: --list_terms='valid_path to list of terms'")

    args            = {}
    logLevel        = rospy.DEBUG if options.debug else rospy.INFO
    rospy.init_node('test_imdb_handler', anonymous=False, log_level=logLevel)

    ## Removing ROS file logging
    root_handlers = logging.getLoggerClass().root.handlers
    if isinstance(root_handlers, list) and len(root_handlers)>0:
        handler = root_handlers[0]
        if type(handler) == logging.handlers.RotatingFileHandler:
            rospy.logwarn("Shutting down file log handler")
            logging.getLoggerClass().root.handlers = []

    ## Defining arguments
    args.update({'list_terms':   options.list_terms})
    
    test_with_db(args)

