#!/usr/bin/python

import nltk
import string
import utilities

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
                                        stop_words='english')

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

  def cosine_sim(self, text1, text2):
    tfidf = self.vectorizer.fit_transform([text1, text2])
    return ((tfidf * tfidf.T).A)[0,1]

  def score(self, base, other):
    try:
      ## Stripping inputs
      other = other.replace(".", " ").strip()
      base = base.strip()

      ## Removing extra spaces in sentences
      base_splitted = base.split()
      base_splitted = ' '.join(base_splitted)
      other_splitted = other.split()
      other_splitted = ' '.join(other_splitted)

      ## Sub-scoring if base is inside other
      complete_phrase = 1.0 if base_splitted in other_splitted else 0.0
      
      ## If both sentences are the same don't do similarity
      if base == other:
          similarity = 1.0
      else:
          similarity = self.cosine_sim(base, other)

      ## Scoring measurements
      score = (complete_phrase+similarity)/2.0
      return score
    except Exception as inst:
      utilities.ParseException(inst)

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