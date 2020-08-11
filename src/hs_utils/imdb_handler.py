#!/usr/bin/env python
# -*- coding: utf-8 -*-

## sudo pip install scipy
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
from imdbpie import Imdb, exceptions
from imdb import IMDb

class IMDbHandler:
    def __init__(self, **kwargs):
        ''' '''
        try: 
            ## search IMDB engine
            self.ia = IMDb()
            
            ## setting up variables
            self.case_sensitive  = []
            self.lower_case      = []
            self.depends_context = []
            self.prefixes        = []
            self.keywords        = []
            self.do_not_include  = []
            
            ## Adding local variables
            self.clean_chars = '[-«\[@#$()\]]'
            self.non_valid  = ['TV episode', 'video game', 'TV short', 'TV series', 'TV mini-series', 'video', 'TV special']
            self.valid = ['feature', 'TV movie']
            self.list_terms = {
                'case_sensitive': [], 
                'prefixes': [], 
                'lower_case': [], 
                'depends_context': [], 
                'do_not_include': [], 
                'keywords': []
            }
            self.comparator = similarity.Similarity()

            # Generating instance of strategy  
            for key, value in kwargs.iteritems():
                if "imdb" == key:
                    self.imdb = Imdb()
                    rospy.logdebug("Created IMDb API handler")
                elif "list_terms" == key:
                    if value is not None:
                        rospy.logdebug("  +   Loading list of terms from [%s]"%value)
                        for file in os.listdir(value):
                            if file.endswith(".txt"):
                                file_path = os.path.join(value, file)
                                key_name = file[:-4]
                                
                                ## split each line in the file
                                splitting = False if key_name == 'do_not_include' else True
                                self.list_terms.update({key_name: self.load_terms(file_path, split_lines=splitting)})
                                rospy.loginfo ("Loaded terms file [%s] with [%d] words"%(key_name, len(self.list_terms[key_name])))

            ## Converting list of termins into UTF-8
            if self.list_terms['case_sensitive']:
                self.case_sensitive = [item.decode('utf8') for item in self.list_terms['case_sensitive']]
            if self.list_terms['lower_case']:
                self.lower_case     = [item.decode('utf8') for item in self.list_terms['lower_case']]
            if self.list_terms['depends_context']:
                self.depends_context= [item.decode('utf8') for item in self.list_terms['depends_context']]
            if self.list_terms['prefixes']:
                self.prefixes       = [item.decode('utf8') for item in self.list_terms['prefixes']]
            if self.list_terms['do_not_include']:
                self.do_not_include = [item.decode('utf8') for item in self.list_terms['do_not_include']]
            if self.list_terms['keywords']:
                self.keywords       = [item.decode('utf8') for item in self.list_terms['keywords']]

        except Exception as inst:
            utilities.ParseException(inst)

    def load_terms(self, fileName, split_lines=True):
        result = None
        try:
            with open(fileName, 'r') as file:
                result = file.read().strip()
                
                if split_lines:
                    result = result.split()
                else:
                    result = result.split("\n")
                
        except Exception as inst:
            utilities.ParseException(inst)
        finally:
            return result

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
    
    def clean_sentence(self, sentence, keywords=[], debug=False):
        def is_prefix(word):
            try:
                for idx, prefix in enumerate(self.prefixes):
                    if word.lower().startswith(prefix.lower() ):
                        return prefix
                return None
            except Exception as inst:
              utilities.ParseException(inst)
                
        def do_not_include(word, search_list):
            result=[]
            try:
                for idx, unwanted in enumerate(search_list):
                    if unwanted in sentence:
                        result.append(unwanted)
                return False
            except Exception as inst:
                utilities.ParseException(inst)
            finally:
                return result

        try:
            pattern = '[&–\[\]@#$+,()]'
            new_sentence    = []
            is_utf          = False
            if self.list_terms is None:
                rospy.logwarn("Invalid list of terms")
                return
            
            ## swapping points if existing
            sentence = sentence.replace('.', ' ').strip()

            ## Removing non-expected characters
            if debug: print "---> sentence:\t ", sentence
            try:
                sentence = sentence.decode('utf-8', 'ignore').strip() 
                sentence = re.sub(pattern, '', sentence)
            except UnicodeDecodeError as inst:
                utilities.ParseException(inst, use_ros=False)
                rospy.logwarn( "Failed character substitution, decoding UTF and ignoring characters")
                try:
                    sentence = sentence.decode('utf8', 'ignore')
                    sentence = re.sub(pattern, '', sentence)
                    rospy.loginfo( "Sentence changed and removed characters")
                    is_utf = True
                except UnicodeDecodeError as inst:
                    rospy.logwarn( "Still failed to show log (decode error)")
                    utilities.ParseException(inst, use_ros=False)

            ## start by removing any word or phrase 
            ## contained in the "do_not_include" list
            not_wanted_words = do_not_include(sentence, self.do_not_include)
            if not_wanted_words:
                for word in not_wanted_words:
                    sentence = sentence.replace(word,'')
                    if debug: print "     0remove:\t ", word
            
            #print "\t::: :::", sentence,"::: :::"
            ## Removing special works from torrent
            ##    and converting if sentence comes
            ##    with strange characters
            if not is_utf:
                splitted    = sentence.strip().split()
            
            splitted    = sentence.strip().split()
            if debug: print "---> 1sentence:\t ", sentence 

            ## Split sentence and look if every word
            ##    is in the special list of characters
            ##    to remove.
            started_bad_section = 0
            for idx, token in enumerate(splitted):
                
                #print "===    token:", token, token == '1'
                if debug: print "     1token:\t ", token
                ## Discarding reserved words for torrent
                ##    typical descriptions
                if token in self.case_sensitive:
                    started_bad_section += 1
                    continue
                if debug: print "       11token:\t ", token
                #print "===    case_sensitive,token:", token, token == '1'
                
                ## Discarding generic lower case
                ##    list of terms
                if token.lower() in self.lower_case:
                    started_bad_section += 1
                    continue
                if debug: print "       12token:\t ", token
                #print "===    lower_case,token:", token, token == '1'
                
                ## remove all tokens that start with
                ## identified prefixes
                if is_prefix( token.lower()) :
                    started_bad_section += 1
                    continue
                if debug: print "       13token:\t ", token
                #print "===    is_prefix,token:", token, token == '1'
                
                ## Keep some of the words for later
                if token in self.keywords:
                    keywords.append(str(token))
                    #print "--- ---keywords:", keywords
                    continue
                if debug: print "       14oken:\t ", token
                #print "===    lower_case,token:", token, token == '1'
                
                ## If sentence already shows bad words
                ##    consider extra words that normally
                ##    would be normal words
                if started_bad_section>1 and token in self.depends_context:
                    started_bad_section += 1
                    continue
                if debug: print "       15token:\t ", token
                
                new_sentence.append(token)
                if debug: print "     2token:\t ", token
                #print "=== new_sentence:", new_sentence
            
            ## Merging list tokens into a string
            try:
                new_sentence = ' '.join(new_sentence)
            except UnicodeDecodeError as inst:
                utilities.ParseException(inst, use_ros=False)
                rospy.logwarn( "Re-making string by decoding UTF and ignoring characters")
                try:
                    new_sentence = [x.decode('utf8', 'ignore') for x in new_sentence]
                    new_sentence = u' '.join(new_sentence)
                except UnicodeDecodeError as inst:
                    rospy.logwarn( "Still failed to show log (decode error)")
                    utilities.ParseException(inst, use_ros=False)
            if debug: print "---> 2new_sentence:\t ", new_sentence
            
            ## Removing non-expected characters
            new_sentence = re.sub('[\[@#$()\]]', '', new_sentence)
            if debug: print "---> 3new_sentence:\t ", new_sentence

        except Exception as inst:
            print "??????!!!!!"
            utilities.ParseException(inst)
        finally:
            return u''.join(new_sentence).encode('utf-8').strip()

    def skim_title(self, torrent_title):
        splitted = None
        year_found = None
        try:

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
                splitted = splitted.translate(None, b"-([_#@&*|~`%^<>]").strip()

            splitted = splitted.decode('utf-8').strip()

        except Exception as inst:
            utilities.ParseException(inst)
        finally:
            return year_found, splitted

    def validate_info(self, record):
        def exists(record, item):
            try:
                if item not in record or not record[item]:
                    rospy.logdebug('  [%s] not found'%item)
                    return False
                return True
            except Exception as inst:
                utilities.ParseException(inst)
                
        try:
            result = True
            result = result and exists(record, 'imdb_id')
            result = result and exists(record, 'genres')
            result = result and exists(record, 'image')
            #result = result and exists(record, 'episodes')
            
            #print "===> duration.result1:", result, exists(record, 'duration')
            result = result and exists(record, 'duration')
            #print "===> duration.result2:", result
            
            result = result and exists(record, 'year')
            result = result and exists(record, 'title')
            result = result and exists(record, 'plot')
            result = result and exists(record, 'rating')
            # result = result or exists(record['rating'], 'old')
            # result = result or exists(record['rating'], 'mature')
            # result = result or exists(record['rating'], 'young')

            #print "===> result:", result
        except Exception as inst:
            utilities.ParseException(inst)
        finally:
            return result
            
    def get_info(self, imdb_id, do_series=None):
        result = None
        try:
            if not imdb_id:
                rospy.logwarn('Invalid IMDB ID')
                return result
            
            #plot_synopsis = self.imdb.get_title_plot_synopsis(imdb_id)
            #images        = self.imdb.get_title_images(imdb_id)
            #ratings       = self.imdb.get_title_ratings(imdb_id)
            title         = self.imdb.get_title(imdb_id)
            genres        = self.imdb.get_title_genres(imdb_id)
            #print "=== original title:"
            
            ## checking if data belongs to an episode
            if 'episode' in title['base']:
                print(' + ID [%s] is an episode'%imdb_id)
                rospy.logwarn('ID [%s] is an episode'%imdb_id)
                parentTitle= title['base']['parentTitle']['id']
                parentId   = parentTitle.strip('/').split('/')[1]
                oldTitle   = title
                title      = self.imdb.get_title(parentId)

            result = {
                'image'        : None,
                'duration'     : None,
                'plot'         : None,
                'episodes'     : None,
                'genres'       : None,
                'rating'       : {
                    'young'    : None,
                    'mature'   : None,
                    'old'      : None,
                    'rating'   : None,
                    'count'    : None,
                },
                'imdb_id'      : imdb_id,
                #'year'         : title['base']['year'],
                'title'        : title['base']['title'],
            
            }
            
            if do_series:
                episodes  = self.imdb.get_title_episodes(imdb_id)
                result['episodes'] = episodes
                info = self.imdb.get_title_episodes_detailed(imdb_id=imdb_id)
                print "- "*20
                pprint(info)
                print "- "*20
            
            ## looking for year
            if 'year' in title['base']:
                result.update({'year': title['base']['year']})

            ## looking for genres
            if 'genres' in genres:
                result['genres'] = genres['genres']
            else:
                rospy.logdebug('  Genres not found')
                
            ## looking for number of episodres
            if 'numberOfEpisodes' in title['base']:
                result['episodes'] = title['base']['numberOfEpisodes']
            else:
                rospy.logdebug('  Number of episode not found')
            
            ## looking for image
            if 'image' in title['base']:
                result['image'] = title['base']['image']
            else:
                rospy.logdebug('  Image not found')

            ## looking for ratings
            if 'ratingCount' in title['ratings']:
                result['count'] = title['ratings']['ratingCount']
            else:
                rospy.logdebug('  Rating count not found')

            if 'rating' in title['ratings']:
                #rating = title['ratings']['rating']
                #print "===> rating:", rating, result['rating']
                result['rating']['rating'] = title['ratings']['rating']
            else:
                rospy.logdebug('  Rating not found')

            if 'ratingsHistograms' in title['ratings']:
                if 'Aged 18-29' in title['ratings']['ratingsHistograms']:
                    result['rating']['young'] = title['ratings']['ratingsHistograms']['Aged 18-29']['aggregateRating']
                else:
                    rospy.logdebug('  Rating of Aged 18-29 not found')
    
                if 'Aged 30-44' in title['ratings']['ratingsHistograms']:
                    result['rating']['young'] = title['ratings']['ratingsHistograms']['Aged 30-44']['aggregateRating']
                else:
                    rospy.logdebug('  Rating of Aged 30-44 not found')
    
                if 'Aged 45+' in title['ratings']['ratingsHistograms']:
                    result['rating']['young'] = title['ratings']['ratingsHistograms']['Aged 45+']['aggregateRating']
                else:
                    rospy.logdebug('  Rating of Aged 45+ not found')
            else:
                rospy.logdebug(' Ratings Histograms not found')
                
            
            ## looking for plot
            if 'plot' in title:
                if 'outline' in title['plot']:
                    result['plot'] = title['plot']['outline']['text']
                elif 'summaries' in title['plot'] and \
                     type(title['plot']['summaries']) is type([]) and \
                     len(title['plot']['summaries'])>0:
                    result['plot'] = title['plot']['summaries'][0]['text']
                else:
                    rospy.logdebug('  Plot text not found')
            else:
                rospy.logdebug('  Plot not found')

            ## looking for running minutes
            if 'runningTimeInMinutes' in title['base']:
                result['duration'] = title['base']['runningTimeInMinutes']
            else:
                rospy.logdebug('  Running time not found')
 
            ## validating resulting message 
            if not self.validate_info(result):
                rospy.logwarn('  Record [%s] is not valid'%imdb_id)
            else:
                rospy.logdebug('  Record [%s] is valid'%imdb_id)
            
            #print "=== images:"
            #pprint(images)
            #print "=== plot_synopsis:"
            #pprint(plot_synopsis)
            #print "=== episodes:"
            #pprint(episodes)
            #print "=== ratings:"
            #pprint(ratings)
            #print "=== genres:"
            #pprint(genres)
            
            if False:
                if 'soundtrack' in title: del title['soundtrack']
                if 'similarities' in title: del title['similarities']
                if 'ratings' in title: del title['ratings']
                if 'filmingLocations' in title: del title['filmingLocations']
                if '@type' in title: del title['@type']
                if 'metacriticScore' in title: del title['metacriticScore']
                
                pprint(title)
                print "-"*80
        except Exception as inst:
            utilities.ParseException(inst)
            pprint(title)
            print "-"*80
            pprint(oldTitle)
        finally:
            return result

    def get_titles(self, imdb, title):
        listed_items = []
        try:
            try:
                imdb_data       = imdb.search_for_title(title)
            except exceptions.ImdbAPIError:
                rospy.logwarn('IMDB limit has been reached while title was searched, waiting for 5 min') 
                time.sleep(300)
                rospy.loginfo('Trying IMDB title search again for [%s]'%title)
                imdb_data       = imdb.search_for_title(title)
            except ValueError:
                try:
                    rospy.logwarn('IMDb did not find titles for [%s]'%title)
                except UnicodeEncodeError as inst:
                    rospy.logwarn('IMDb did not find titles')
                    utilities.ParseException(inst, use_ros=False)
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
                score = self.comparator.score(splitted_title, imdb_item['title'], debug=debug)
                year_matches    = year_found == imdb_item['year']
                item_type       = 'feature' == imdb_item['type']
                imdb_item.update({'score':score})
                imdb_item.update({'year_matches':year_matches})
                imdb_item['title'] = u''.join(imdb_item['title']).encode('utf-8').strip()
                
                ## Adding only very similar titles
                if score > 0.98:
                    updated_imdb.append(imdb_item)
                else:
                    ignored_items      += 1
                    try:
                        imdb_title_decoded = imdb_item['title'].decode('utf-8').strip()
                        splitted_title = splitted_title.decode('utf-8').strip()
                        rospy.logdebug("      Ignored [%s] vs [%s] = %f"%
                                      (splitted_title, imdb_title_decoded, score))
                    except UnicodeEncodeError as inst:
                        utilities.ParseException(inst, use_ros=False)
                        rospy.logwarn( "Log message missing, encoding UTF and ignoring characters")
                        try:
                            rospy.loginfo( "Failed to show splitted decoded sentence [%s]"%splitted_title)
                        except UnicodeDecodeError as inst:
                            rospy.logwarn( "Still failed to show log (decode error)")
                            utilities.ParseException(inst, use_ros=False)
                        except UnicodeEncodeError as inst:
                            rospy.logwarn( "Still failed to show log (encode error)")
                            utilities.ParseException(inst, use_ros=False)
                        
            rospy.logdebug("+   Ignored [%s] item(s)"%(str(ignored_items)))
            
            ## If all items are ignored take the highest one
            if len(imdb_data)<1:
                return
                
            if total_items == ignored_items:
                sorted_imdb = sorted(imdb_data, key=itemgetter('score'), reverse=True) 
                try:
                    rospy.loginfo("Taking highest score for [%s]"%splitted_title)
                except UnicodeEncodeError as inst:
                    rospy.logwarn( "Log message missing, encoding UTF and ignoring characters (encode error)")
                    utilities.ParseException(inst, use_ros=False)
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
                    except LookupError:
                        #utilities.ParseException(inst)
                        rospy.logwarn('IMDB title genres not found for [%s] with [%s]'%
                                      (imdb_item['title'], imdb_id) )
                    except exceptions.ImdbAPIError:
                        rospy.logwarn('IMDB limit has been reached while searching genres, waiting for 5 min') 
                        time.sleep(300)
                        rospy.loginfo('Trying IMDB genre search again for [%s]'%title)
                        title_genres        = imdb.get_title_genres(imdb_id)
                    
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

            item_selected.update({'imdb_info':imdb_selected})
            item_selected.update({'query_title':splitted_title})
        except Exception as inst:
          utilities.ParseException(inst)
        finally:
            return item_selected

def search_extra(title):
    try:
        list_terms = '/home/renato/workspace/projects/home_services/src/imdb_collector/config/'
        args = {'list_terms':   list_terms}
        ih = IMDbHandler(**args)
        print "QUERY:", title
        data = ih.ia.search_movie_advanced(title)
        print "RESULT:"
        pprint(data)
    except Exception as inst:
        utilities.ParseException(inst)

def search_title(args):
    try:
        
        title = args['title']
        list_terms = '/home/renato/workspace/projects/home_services/src/imdb_collector/config/'
        args = {'list_terms':   list_terms}
        ih = IMDbHandler(**args)
        year_found=None
        data = ih.get_imdb_best_title(title, year_found=year_found)

        pprint(data)
    except Exception as inst:
        utilities.ParseException(inst)

def search(args):
    try:
        imdb_id = args['imdb_id']
        imdb_handler = IMDbHandler(**args)
        print("  + Created IMDb handler")
        print("  + Searching for [%s]"%imdb_id)
        
        series = args['do_series'] if args['do_series'] else None
        data   = imdb_handler.get_info(imdb_id, do_series=series)
        pprint(data)
    except Exception as inst:
        utilities.ParseException(inst)

def clean_title(title, debug=False):
    #s = "Institute (Gecko Theatre Company) (1280x720p HD, 50fps, soft Eng subs)"
    list_terms = '/home/renato/workspace/projects/home_services/src/imdb_collector/config/'
    list_terms = '/opt/home_services/src/home_services/src/imdb_collector/config'
    args = {
        'list_terms':list_terms,
        'imdb':      True
    }
    imdb_handler = IMDbHandler(**args)
    keywords     = []
    #print "CLEANING:", title
    mopped = imdb_handler.clean_sentence(title, keywords, debug=debug).strip(' -')
    #print "RESULT:"
    #print "\t",mopped
    #print "\t",keywords
    print(mopped)

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
                
                #pprint(items)
                print "-"*80
                counter += 1
            # Allow ROS to go to all callbacks.
            #rospy.spin()
        except rospy.ROSInterruptException:
            pass
    except Exception as inst:
        utilities.ParseException(inst)

if __name__ == '__main__':
    #logging.getLogger('imdbpie').setLevel(logging.getLevelName('DEBUG'))
    #logging.getLogger('imdbpie.imdbpie').setLevel(logging.getLevelName('DEBUG'))
    #logging.getLogger('imdbpy.parser.http').setLevel(logging.getLevelName('DEBUG'))
    
    usage       = "usage: %prog option1=string option2=bool"
    parser      = OptionParser(usage=usage)
    parser.add_option('--debug', '-d',
                action='store_true',
                default=False,
                help='Input debug level')
    parser.add_option('--list_terms',
                type="string",
                action='store',
                default=None,
                help='Input list of term path')
    parser.add_option('--std_out', '-o',
                action='store_false',
                default=True,
                help='Allowing standard output')
    operations = OptionGroup(parser, "Inline operations",
                    "Use to search directly from IMDB data")
    operations.add_option('--search','-s',
                type="string",
                action='store',
                default=None,
                help='Search for IMDB ID')
    operations.add_option('--episodes','-e',
                action='store_true',
                default=False,
                help='Search for episode information')
    operations.add_option('--search_title', '-t',
                type="string",
                action='store',
                default=None,
                help='Search for best title')
    operations.add_option('--clean_title', '-c',
                type="string",
                action='store',
                default=None,
                help='Clean title')
    operations.add_option('--search_extra',
                type="string",
                action='store',
                default=None,
                help='Search extra IMDB title')
    operations.add_option('--db',
                action='store_true',
                default=False,
                help='Test local DB')
    parser.add_option_group(operations)
    (options, args) = parser.parse_args()
    
    args            = {}
    args.update({'allow_std_out': options.std_out})
    args.update({'do_series':     options.episodes})
    args.update({'imdb':          True})
    
    if options.search is not None:
        args.update({'imdb_id':       options.search})
        search(args)
        
    if options.search_title is not None:
        args.update({'title':  options.search_title})
        search_title(args)
        
    if options.clean_title is not None:
        clean_title(options.clean_title, debug=options.debug)
        
    if options.search_extra is not None:
        search_extra(options.search_extra)
        
    if options.db:
        print "Testing IMDB in local database"
        if options.list_terms is None:
           parser.error("Missing required option: --list_terms='valid_path to list of terms'")

        args            = {}
        logLevel        = rospy.DEBUG if options.debug else rospy.INFO
        rospy.init_node('test_imdb_handler', anonymous=False, log_level=logLevel)
    
        ## Defining arguments
        args.update({'list_terms':    options.list_terms})        
        test_with_db(args)
    
