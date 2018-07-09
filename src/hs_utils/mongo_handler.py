#!/usr/bin/env python
import sys, os
import time
import json
import pymongo
import ast
import datetime
import pprint
import rospy

from optparse import OptionParser, OptionGroup
from pymongo import MongoClient
from pymongo.errors import ConnectionFailure

import ros_node

sys.setrecursionlimit(5000)
class MongoAccess:
  def __init__(self, debug=2):
    ''' '''
    self.component	   = self.__class__.__name__
    self.coll_name     = None
    self.collection    = None
    self.db	 	       = None
    self.debug	 	   = debug

    if self.debug == 0:    
        logType = 'ROSPY'
    elif self.debug == 1:
        logType = 'ON'
    else:
        logType = 'OFF'
    self.Debug("Creating mongo client with debug mode [%s]"%logType)
    
  def Connect(self, database, collection, host='localhost', port=27017):      
    try: 
        return self.connect(database, collection, host=host, port=port)
    except Exception as inst:
      ros_node.ParseException(inst)

  def connect(self, database, collection, host='localhost', port=27017):
    ''' '''
    result = False
    if database is None:
        self.Debug("Error: Invalid database name")
    try: 
      self.Debug("Generating instance of mongo client")
      # Creating mongo client
      client = MongoClient(host, port)
      
      try:
        # The ismaster command is cheap and does not require auth.
        client.admin.command('ismaster')
      except ConnectionFailure:
        self.Debug("Error: Server not available")
        return result

      # Getting instance of database
      self.Debug("Getting instance of database [%s]"%database)
      self.db = client[database]

      # Getting instance of collection
      self.Debug("Getting instance of collection")
      self.collection = self.db[collection]
      self.coll_name = collection
      
      result = self.collection is not None
    
    except Exception as inst:
      ros_node.ParseException(inst)
    finally:
      return result
      
  def Insert(self, document):
    post_id = None
    try: 
        self.Debug("Inserting document in collection [%s]"%(self.coll_name))
        post_id = self.collection.insert(document)
    except Exception as inst:
        ros_node.ParseException(inst)
    finally:
        return post_id

  def Find(self, condition={}, timeout=False):
    '''Collects data from database '''
    posts = None
    try: 
      self.Debug("Finding document in collection [%s]"%(self.coll_name))
      posts = self.collection.find(condition, no_cursor_timeout=timeout)
    except Exception as inst:
      ros_node.ParseException(inst)
    return posts
    
  def Print(self, posts, with_id=False):
      '''Prints collection of posts '''
      try: 
          if not isinstance(posts,type(None)):
              sizePosts= posts.count()
              for i in range(sizePosts):
                  post = posts[i]
                  postKeys = post.keys()
                  line = ''
                  for key in postKeys:
                      if not with_id and key=='_id':
                          continue
                      line += ('{'+key+': '+str(post[key])+"} ")
                  line = line.strip()
                  self.Debug('  '+str(line))
          else:
              self.Debug("Invalid input posts for printing")
      except Exception as inst:
          ros_node.ParseException(inst)

  def Remove(self, condition=None):
    '''Deletes data from database '''
    result = None
    if condition is not None:
      
      if len(condition)<1:
	self.Debug("Deleting all documents from collection")
	
      posts = self.collection.find(condition)
      ## Deleting documents
      result = self.collection.remove(condition)
      if result['ok']>0:
	self.Debug("  Deleted %s documents in %s, s"% (str(result['n']), str(result['syncMillis'])))
      else:
	self.Debug("  Deleted failed: %s", str(result))

    else:
      self.Debug("Invalid condition for deleting")
    return result

  def Size(self):
    collSize = None
    try: 
      if self.collection is not None:
        collSize = self.collection.count()
        self.Debug("Collection [%s] has a size of [%s]"%(self.coll_name, str(collSize)))
    except Exception as inst:
      ros_node.ParseException(inst)
    finally:
      return collSize
 
  def Update(self, condition=None, substitute=None, upsertValue=False):
    '''
    Updates data from database. 
    condition    dictionary with lookup condition
    substitue    item to substitute
    upsertValue    True for insert if not exist, False otherwise
    
    returns True if update was OK, otherwise False
    '''
    result = False
    try: 
        if condition is None or len(condition)<1:
            self.Debug("Error: Invalid given condition" )
            return
            
        if substitute is None:
            self.Debug("Error: Invalid given substitute" )
            return
        
        ## if '_id' not in substitute.keys():
        ##     self.Debug("Warning: not given _id in substitute part")
        ## else:
        ##     substitute.pop('_id', 0)
        upsert_label = "existing "
        if upsertValue:
            upsert_label = "and creating new "
        
        self.Debug("Updating %sdocuments from collection [%s]"%
                          (upsert_label, self.coll_name))
        resultSet = self.collection.update(condition,
                                           {'$set': substitute}, 
                                           upsert=upsertValue, 
                                           multi=False)
        
        result = resultSet['ok'] == 1
        if not result:
            self.Debug("Warning: Not expected result set:\n%s"%
                              str(resultSet))
    except Exception as inst:
      utilities.ParseException(inst)
    finally:
      return result

  def Debug(self, line):
    try: 
        if self.debug == 0:
            rospy.logdebug(line)
        elif self.debug == 1:
            print(line)
    except Exception as inst:
        utilities.ParseException(inst)

def db_handler_call(options):
  ''' Method for calling MongoAccess handler'''
  
  try:
    database = MongoAccess(debug=1)
    database.Connect(options.database, options.collections)
    
    if options.insert:
      post_id = database.Insert(options.document)
      if post_id is not None:
        database.Debug("Item inserted with ID: %s"%(str(post_id)))
  
    elif options.report and options.slice:
      print("Finding documents from %d to %d"%(options.start, options.end))
      posts   = database.Find(options.condition)
      entries = posts[options.start:options.end]
      counter = options.start
      for entry in entries:
        print "["+str(counter)+"]"+str(entry['_id'])
        counter += 1

    elif options.report:
      posts = database.Find(options.condition)
      database.Print(posts, with_id=options.with_ids)
	
    elif options.delete:
      result = database.Remove(condition=options.removal)
      
    elif options.update:
      result = database.Update(condition=options.referal, 
			       substitute=options.replace)

  except Exception as inst:
    utilities.ParseException(inst)
  
if __name__ == '__main__':
  ''''''
  print('Logger created.')

  usage = "usage: %prog opt1=arg1 opt2=arg2"
  parser = OptionParser(usage=usage)
  parser.add_option("-d", '--database', 
		      metavar="DB", 
                      dest='database',
		      default=None,
		      help="Database name")
  parser.add_option("-c", '--collections', 
		      metavar="CATALAGUE", 
                      dest='collections',
		      default=None,
		      help="Database collections")
  
  dbOperations = OptionGroup(parser, "Database insert operations",
		      "These options are for using handler operations")
  dbOperations.add_option("-i", "--insert",
		      action="store_true", 
		      dest="insert", 
		      default=False,
		      help="Inserts an item")
  dbOperations.add_option('--document', 
		      metavar="JSON", 
                      dest='document',
		      default='',
		      help="Document to insert in text format")
  
  findOps = OptionGroup(parser, "Database finding operations",
		      "These options are for using handler operations")
  findOps.add_option("-l", "--report",
		      action="store_true", 
		      dest="report", 
		      default=False,
		      help="Reports items")
  findOps.add_option('--condition', 
		      metavar="JSON", 
                      dest='condition',
		      default='',
		      help="Search condition to find documents")
  findOps.add_option("--with_ids",
		      action="store_true", 
		      dest="with_ids", 
		      default=False,
		      help="Set this option to print object IDs")
  
  sliceOperations = OptionGroup(parser, "Database find operations with document slicing",
              "These options are for giving chunks of entries")
  sliceOperations.add_option("-s", "--slice",
              action="store_true", 
              dest="slice", 
              default=False,
              help="Slices query")
  sliceOperations.add_option('--start',
              type="int",
              action='store',
              default=None,
              help="Slices starting index")
  sliceOperations.add_option('--end',
              type="int",
              action='store',
              default=None,
              help="Slices ending index")
  
  delOps = OptionGroup(parser, "Database deleting operations",
		      "These options are for using handler operations")
  delOps.add_option("-r", "--delete",
		      action="store_true", 
		      dest="delete", 
		      default=False,
		      help="Delete items")
  delOps.add_option('--removal', 
		      metavar="JSON", 
                      dest='removal',
		      default='',
		      help="Condition to delete documents")
  
  updateOps = OptionGroup(parser, "Database updating operations",
		      "These options are for using handler operations")
  updateOps.add_option("-u", "--update",
		      action="store_true", 
		      dest="update", 
		      default=False,
		      help="Update items")
  updateOps.add_option('--referal', 
		      metavar="JSON", 
                      dest='referal',
		      default='',
		      help="Condition to update documents")
  updateOps.add_option('--replace', 
		      metavar="JSON", 
                      dest='replace',
		      default='',
		      help="Substitute document for update method")
  
  parser.add_option_group(dbOperations)
  parser.add_option_group(findOps)
  parser.add_option_group(delOps)
  parser.add_option_group(updateOps)
  parser.add_option_group(sliceOperations)
  
  (options, args) = parser.parse_args()
  #print (options)
  
  printHelp = False
  if options.database is None:
    parser.error("Missing required option: database")
    printHelp = True
    
  if options.collections is None:
    parser.error("Missing required option: collections")
    printHelp = True
    
  if options.insert:
    if len(options.document)<1:
      parser.error("Missing required INSERT option: document")
    else:
      options.document = ast.literal_eval(options.document)
    
  elif options.report:
    if len(options.condition)<1:
      options.condition = {}
    else:
      options.condition = ast.literal_eval(options.condition)
      
    if options.slice:
      if options.start is None:
        parser.error("Missing required SLICE option: --start=0")
      if options.end is None:
        parser.error("Missing required SLICE option: --end=5")
      
  elif options.delete:
    if len(options.removal)<1:
      parser.error("Missing required DELETE option: removal")
    else:
      options.removal = ast.literal_eval(options.removal)
	
  elif options.update:
    if len(options.referal)<1:
      parser.error("Missing required UPDATE option: referal")
    if len(options.replace)<1:
      parser.error("Missing required UPDATE option: replace")
    else:
      options.referal = ast.literal_eval(options.referal)
      options.replace = ast.literal_eval(options.replace)
  else:
      print("No option was selected")
      sys.exit(0)

  if printHelp:
    parser.print_help()
  
  db_handler_call(options)
