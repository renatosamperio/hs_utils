#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import nltk
import string
import utilities
import rospy
import re

from optparse import OptionParser
from sklearn.feature_extraction.text import TfidfVectorizer

nltk.download('punkt') # if necessary...

class Similarity:
  def __init__(self, **kwargs):
    try:
      self.stemmer = nltk.stem.porter.PorterStemmer()
      self.remove_punctuation_map = dict((ord(char), None) 
                                        for char in string.punctuation)
      self.vectorizer = TfidfVectorizer(tokenizer=self.normalize, 
                                        stop_words=None)

      self.others = None
      self.base = None
      # Generating instance of strategy 
      for key, value in kwargs.iteritems():
        if "base" == key:
          self.base = value
        elif "others" == key:
          self.others = value
    except Exception as inst:
      utilities.ParseException(inst)

  def stem_tokens(self, tokens):
    return [self.stemmer.stem(item) for item in tokens]

  def normalize(self, text):
    '''remove punctuation, lowercase, stem'''
    return self.stem_tokens(
              nltk.word_tokenize(
                  text.lower().translate(
                      self.remove_punctuation_map)))

  def cosine_sim(self, text1, text2, debug=False):
    try:
        tfidf = self.vectorizer.fit_transform([text1, text2])
        cosine_sim =  ((tfidf * tfidf.T).A)[0,1]
        return cosine_sim
    except Exception as inst:
        utilities.ParseException(inst)
        raise Exception('Error in cosine similarity!') 

  def score(self, base, other, debug=False):
    try:
        ## Stripping inputs
        oring_base = base
        oring_other = other
        base = base.replace(".", " ").strip()
        base = base.replace("-", " ").strip()
        other = other.replace(".", " ").strip()
        other = other.replace("-", " ").strip()
        
        ## Removing extra spaces in sentences
        base_splitted = base.split()
        base_splitted = u' '.join(base_splitted).encode('utf-8').strip()
        base_splitted = base_splitted.translate(None, b"':([_#@&*|~`%^<>").strip()
        base_splitted = re.sub('-.', ' ', base_splitted)
#         base_splitted = base_splitted.replace('-', ' ')
#         base_splitted = base_splitted.replace('.', ' ')
        
        other_splitted = other.split()
        other_splitted = u' '.join(other_splitted).encode('utf-8').strip()
        other_splitted = other_splitted.translate(None, b"':([_#@&*|~`%^<>").strip()
        other_splitted = re.sub('-.', ' ', other_splitted)
#         other_splitted = other_splitted.replace('-', ' ')
#         other_splitted = other_splitted.replace('.', ' ')
        
        ## Sub-scoring if base is inside other
        has_same_words = len(base_splitted) == len(other_splitted)
        is_inside_other= base_splitted.lower() in other_splitted.lower() or other_splitted.lower() in base_splitted.lower()
        
        if debug:
            rospy.loginfo("===> base:\t\t%s"%base)
            rospy.loginfo("===> other:\t\t%s"%other)
            rospy.loginfo("===> base_splitted:\t%s"%base_splitted)
            rospy.loginfo("===> other_splitted:\t%s"%other_splitted)
            rospy.loginfo("===> is_inside_other:\t%s"%is_inside_other)
            rospy.loginfo("===> has_same_words:\t%s"%has_same_words)
        
        ## Splitting educated guess with inputs that are
        ##    self contained and have same number of words
        complete_phrase = 0.5 if has_same_words else 0.0
        complete_phrase += 0.5 if is_inside_other else 0.0
        
        ## If both sentences are the same don't do similarity
        if base == other:
            similarity = 1.0
        else:
            base_lower = base.lower()
            other_lower = other.lower()
            if debug:
                rospy.loginfo("===> base.lower:\t%s"%base_lower)
                rospy.loginfo("===> other.lower:\t%s"%other_lower)
            try:
                measure1 = self.cosine_sim(base_lower, other_lower, debug=True)
            except Exception:
                print("Error: cosine similarity failed with [%s] and [%s]"%
                              (str(base_lower), str(other_lower)))
                measure1 = self.cosine_sim(oring_base, oring_other)
                
            try:
                measure2 = self.cosine_sim(oring_base, oring_other)
            except ValueError:
                base_splitted = [item for item in oring_base.split()]
                other_splitted = [item for item in oring_other.split()]
                measure2 = self.cosine_sim(oring_base.split(), oring_other.split())
                if debug:
                    rospy.loginfo("===> measure1:\t\t%s"%measure1)
                    rospy.loginfo("===> measure2:\t\t%s"%measure2)
            ## Sometimes the original inputs have better
            ##    similarity than skimmed strings
            similarity = max(measure1, measure2)
            if similarity is None:
                score = -1
                return
        
        ## Scoring measurements
        score = (complete_phrase+similarity)/2.0
        
        if base_splitted == other_splitted:
            similarity2 = 1.0
        else:
            similarity2 = self.cosine_sim(base_splitted, other_splitted)
         
        if debug:
            rospy.loginfo("===> complete_phrase:\t%s"%complete_phrase)
            rospy.loginfo("===> similarity:\t%s"%similarity)
            rospy.loginfo("===> similarity2:\t%s"%similarity2)
            rospy.loginfo("")
#         score2 = (complete_phrase+similarity2)/2.0
#         print "===> score:\t\t", score
#         print "===> score2:\t\t", score2

    except Exception as inst:
        utilities.ParseException(inst)
    finally:
        return score

def example4(task):
  try:

    base = "Tron"
    others = [
    "Pain and Gain 2013 720p WEBRip x264 AC3-TRON",
    "Avengers.Age.of.Ultron.2015.1080p.3D.BluRay.Half-SBS.x264.DTS-HD.MA.7.1-RARBG",
    "Gra o Tron Sezon 1 720p.BDRip.XviD.AC3-ELiTE Lektor PL Pawulon (krzysiekvip2)",
    "Gra o Tron Sezon 2 720p.BDRip.XviD.AC3-ELiTE Lektor PL Pawulon (krzysiekvip2)",
    "TRON Legacy 2010 [English] DVDRip (XViD)",
    "Avengers.Age.of.Ultron.2015.1080p.BluRay.REMUX.AVC.DTS-HD.MA.7.1-RARBG",
    "Gra o Tron Sezon 3 720p.BDRip.XviD.AC3-ELiTE Lektor PL Pawulon (krzysiekvip2)",
    "Stronger.2017.1080p.BluRay.REMUX.AVC.DTS-HD.MA.5.1-FGT",
    "Il Trono di Spade - Game of Thrones -Stagione 1 [HDTVMux720p.Ita][Nautilus-BT] (PittaSk8)",
    "Avengers.Age.of.Ultron.2015.1080p.3D.BluRay.AVC.DTS-HD.MA.7.1-RARBG",
    "Tron.1982.Bluray.1080p.DTS-HD.x264-Grym (vonRicht)",
    "Game of Thrones - S03 Complete - 1080p x264 ENG-ITA BluRay - Il Trono Di Spade (2013) (ShivaShanti2)",
    "WII Tron Evolution Battle Grids n Space Disney Interactive Studios PAL ESPALWII ..",
    "Snitch 2013 720p HDRip AAC-TRON",
    "FAST & FURIOUS 2009 DVDRIP DALE-0-TRON(TRN)",
    "Tron.Legacy.2010.BluRay.1080p.DTS.x264-CHD (dontamil)",
    "Tron.Legacy.2010.Bluray.1080p.DTS-HD7.1.x264-Grym (vonRicht)"
    ]

    print ("Base: %s"%base)
    for other in others:
      print("\t %s : %s"%( str(task.score(base, other)) , str(other)))
  except Exception as inst:
    utilities.ParseException(inst)

def example3(task):
  try:

    base = "Star Wars The Last Jedi"
    others = [
    "Star Wars The Last Jedi (2017) Untouched HINDI CAM MovCR com Exc",
    "Star Wars The Last Jedi 2017 FULL HDCAM ENGLiSH x264 HQMIC-DADDY (Silmarillion)",
    "Star Wars The Last Jedi 2017 FULL HDCAM ENGLiSH x264 HQMIC-DADDY (MrStark)",
    "Star Wars The Last Jedi 2017 FULL HDCAM ENGLiSH x264 HQMIC-DADDY  (MrStark)",
    "Star Wars The Last Jedi 2017 FULL HDCAM ENGLiSH x264 HQMIC-DADDY (mazemaze16)",
    "Star.Wars.The.Last.Jedi.2017.CAM.X264-BebeLeitinho (Supernova)",
    "Star.Wars.The.Last.Jedi.2017.CAM.XViD-26k (mazemaze16)",
    "Star.Wars.The.Last.Jedi.HDCAM.720p.x264.Korean.Hardcoded (Anonymous)",
    "Star Wars The Last Jedi HDCAM 720p x264 Korean Hardcoded",
    "8  Star Wars The Last Jedi 2017 TS 720.."
    ]

    print ("Base: %s"%base)
    for other in others:
      print("\t %s : %s"%( str(task.score(base, other)) , str(other)))
  except Exception as inst:
    utilities.ParseException(inst)

def example2(task):
  try:
    base = "Star Wars Episode 8"
    others = [
    "Star Wars Episode III - Revenge of the Sith 2005 BluRay 1080p DTS LoNeWolf (bobnjeff)",
    "Star Wars Episode II - Attack of the Clones 2002 BluRay 1080p DTS LoNeWolf (bobnjeff)",
    "(Fan Edit) Star Wars Episode VII: The Force Awakens - Restructured V2 1080p (kirkafur)",
    "Star Wars: Episode V - The Empire Strikes Back (1980) 1080p BrRip x264 - YIFY",
    "Star Wars Episode VII The Force Awakens 2015 1080p BluRay x264 DTS-JYK",
    "Star Wars: Episode II - Attack of the Clones (2002) 1080p BrRip x264 - YIFY",
    "Star Wars: Episode VI - Return of the Jedi (1983) 1080p BrRip x264 - YIFY",
    "Star Wars: Episode VII - The Force Awakens (2015) 1080p BluRay - 6CH - 2 5GB - S..",
    "Star Wars Episode II Attack Of The Clones 2002 MULTI UHD 4K x264 (SaM)",
    "Star.Wars.Episode.VII.The.Force.Awakens.2015.1080p.BluRay.REMUX. (Anonymous)"
    ]

    print ("Base: %s"%base)
    for other in others:
      print("\t %s : %s"%( str(task.score(base, other)) , str(other)))
  except Exception as inst:
    utilities.ParseException(inst)

def example1(task):
  try:
    base = "Blade Runner 2049"
    others = [
    "Blade Runner 2049 2017 NEW HD-TS X264 HQ-CPG (xxxlavalxxx)",
    "Blade Runner 2049.1080p.WEB-DL.H264.AC3-EVO (mazemaze16)",
    "Blade Runner 2049 2017 NEW HD-TS X264 HQ-CPG (makintos13)",
    "Blade Runner 2049 2017 1080p WEBRip 6CH AAC x264 - EiE (samcode4u)",
    "Blade Runner 2049 2017 FULL CAM x264 - THESTiG (makintos13)",
    "Blade Runner 2049 2017 1080p WEB-DL DD5.1 x264-PSYPHER (ViVeKRaNa)",
    "Blade Runner 2049 720p WEB-DL H264 AC3-EVO[EtHD]",
    "Blade Runner 2049 (2017) 1080p WEB-DL 6CH 2.7GB - MkvCage (MkvCage)",
    "Blade Runner 2049 2017 1080p WEB-DL DD5 1 H264-FGT",
    "Blade Runner 2049 1080p WEB-DL X264 6CH HQ-CPG",
    "Blade Runner 2049 1080p WEB-DL H264 AC3-EVO[EtHD]",
    "Blade Runner 2049 2017 iTA ENG MD-AC3 WEBDL 1080p x264-BG mkv",
    "Blade Runner 2049 2017 1080p WEB-DL x265 HEVC 6CH-MRN (MRNTUT)"
    ]

    print ("Base: %s"%base)
    for other in others:
      print("\t %s : %s"%( str(task.score(base, other)) , str(other)))
  except Exception as inst:
    utilities.ParseException(inst)

def call_task():
  ''' Command line method for running sniffer service'''
  try:
    
    args = {}
    #base, others = example1()
    #args.update({'base': base})
    #args.update({'others': others})

    taskAction = Similarity(**args)
    example1(taskAction)
    example2(taskAction)
    example3(taskAction)
    example4(taskAction)

  except Exception as inst:
    utilities.ParseException(inst)

if __name__ == '__main__':    
  call_task()