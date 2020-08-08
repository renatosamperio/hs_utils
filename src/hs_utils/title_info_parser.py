#!/usr/bin/env python

import re
import rospy
import logging

from pprint import pprint
from collections import defaultdict
from optparse import OptionParser, OptionGroup

from hs_utils import ros_node

class TitleInfoParser:
    def __init__(self, **kwargs):
        try:
            self.pattens = {
                'parts':         '(\d{1,2}(of+?)\d{1,2})',
                'episode_info':  r"(?:s|season)(\d{2})(?:e|x|episode|\n)(\d{2})",
                'date':          r'(\d{4}[+\s]\d{2}[+\s]\d{2})',
                'ext_date':      "(\d{2}\D|$).[+\s](?:Jan(?:uary)?|Feb(?:ruary)?|Mar(?:ch)?|Apr(?:il)?|May|Jun(?:e)?|Jul(?:y)?|Aug(?:ust)?|Sep(?:tember)?|Oct(?:ober)?|(Nov|Dec)(?:ember)?)[+\s](?:19[7-9]\d|2\d{3})",
                'season_part':   '((Series[+\s](\d{1,2})*)*)([+\s]*(Part(e?)[+\s](\d{1,2}|[a-zA-Z]+)))',
                'single_season': '((S(\d{2})[+\s]*)+(-.*|$|[+\s]*)+)(E(\d{2})|$)*',
                'single_date':   '(\d{4}[+\s]?)((\d{2})|$)?',
                'single_series': '(Series[+\s](\d{1,2})*)([\s]?|$)',
                'single_episode': '(E(\d{2}).+([+\s]?|$))'
            }

            
            for key, value in kwargs.iteritems():
                if "pattens" == key:
                    self.pattens = value
                    rospy.logdebug('  Got [%d] patterns'%len(self.pattens))
            
            ## Creating pattern reader
            rospy.logdebug('  Creating pattern reader')
            self.read = defaultdict(lambda:[]) 
            self.read.update({
                'episode_info': 
                    lambda title: 
                        self.get_episode_info(self.pattens['episode_info'], title),
                'date': 
                    lambda title: 
                        self.get_date(self.pattens['date'], title),
                'ext_date': 
                    lambda title: 
                        self.get_ext_date(self.pattens['ext_date'], title),
                'parts': 
                    lambda title:
                        self.get_parts(self.pattens['parts'], title),
                'season_part': 
                    lambda title:
                        self.get_season_part(self.pattens['season_part'], title),
                'single_season': 
                    lambda title:
                        self.get_single_season(self.pattens['single_season'], title),
                'single_date': 
                    lambda title:
                        self.get_single_date(self.pattens['single_date'], title),
                'single_series': 
                    lambda title:
                        self.get_single_series(self.pattens['single_series'], title),
                'single_episode': 
                    lambda title:
                        self.get_single_episode(self.pattens['single_episode'], title),
            })
            
            ## Defining tags for iterating on patterns
            #self.tags = self.read.keys()
            self.tags = {
                0: ['episode_info', 
                    'date', 
                    'ext_date', 
                    'parts', 
                    'season_part'
                ],

                1: [
                    'single_season', 
                    'single_date',
                    'single_series',
                    'single_episode'
                ]
            } 

            rospy.logdebug('  Created title info parser')

        except Exception as inst:
            ros_node.ParseException(inst)

    def get_episode_info(self, pattern, mopped):
        try:
            has_info = re.search(pattern, mopped, re.I)
            if has_info:
                rospy.logdebug('  Looking for episode info')
                start,end      = has_info.span() 
                mopped         = mopped[:start]+ mopped[end:]
                mopped         = mopped.strip()
                season,episode = has_info.groups()
                
                return  {
                    'title':  mopped,
                    'season': season,
                    'episode':episode
                }
            return None

        except Exception as inst:
              ros_node.ParseException(inst)

    def get_date(self, pattern, mopped):
        try:
            
            has_info = re.search(pattern, mopped, re.I)
            if has_info:
                rospy.logdebug('  Looking for date')
                start,end   = has_info.span() 
                mopped      = mopped[:start]+ mopped[end:]
                mopped      = mopped.strip()
                date_torrent= has_info.groups()[0]
                
                return  {
                    'title':  mopped,
                    'date':date_torrent
                }
            return None
        except Exception as inst:
              ros_node.ParseException(inst)

    def get_ext_date(self, pattern, mopped):
        '''
        
            pattern_ext_date = ""+ \
            "(\d{2}\D|$)." + \
            "[+\s]" + \
            "(?:" + \
                "Jan(?:uary)?|" + \
                "Feb(?:ruary)?|" + \
                "Mar(?:ch)?|" + \
                "Apr(?:il)?|" + \
                "May|" + \
                "Jun(?:e)?|" + \
                "Jul(?:y)?|" + \
                "Aug(?:ust)?|" + \
                "Sep(?:tember)?|" + \
                "Oct(?:ober)?|" + \
                "(Nov|" + \
                "Dec)(?:ember)" + \
            "?)" + \
            "[+\s]" + \
            "(?:" + \
              "19[7-9]\\d|2\\d{3}" + \
            ")"
        '''
        try:
            
            has_info = re.search(pattern, mopped, re.I)
            if has_info:
                rospy.logdebug('  Looking for extended date')
                start,end   = has_info.span()
                date_torrent= mopped[start:end]
                groups      = mopped[start:end]
                mopped      = mopped[:start]+ mopped[end:]
                mopped      = mopped.strip()
                date_torrent= date_torrent.strip()
                
                return  {
                    'title':  mopped,
                    'date':date_torrent
                }
            return None

        except Exception as inst:
              ros_node.ParseException(inst)

    def get_parts(self, pattern, mopped):
        try:
            has_info = re.search(pattern, mopped, re.I)
            if has_info:
                rospy.logdebug('  Looking for parts sections')
                start,end = has_info.span()
                section   = mopped[start:end].strip()
                mopped    = mopped[:start]+ mopped[end:]
                mopped    = mopped.strip()
                part,total_parts= section.split('of')
                
                return  {
                    'title':      mopped,
                    'part':       part,
                    'total_parts':total_parts,
                }
            return None
        except Exception as inst:
              ros_node.ParseException(inst)
              
    def get_season_part(self, pattern, mopped):
        try:
            
            has_info = re.search(pattern, mopped, re.I)
            if has_info:
                rospy.logdebug('  Looking for season/part explanation')
                
                start,end  = has_info.span()
                sections   = mopped[start:end]
                mopped     = mopped[:start]+ mopped[end:]
                mopped     = mopped.strip()
                season,part='',''
                
                ## this regex can give only the Part side
                ## therefore, season has to be dismissed
                if 'Part' in sections:
                    split_idx = sections.find('Part')

                    ## we need to split result into season
                    ## and part sections. 
                    if split_idx>0:
                        season = sections[:split_idx].strip()
                        part   = sections[split_idx:].strip()
                    
                    ## assign season section only
                    else:
                        season = sections
                ## assign part section only 
                else:
                    part = sections
                
                ## edit sections in different stages,
                ## adding only the part section
                result = {
                    'title':  mopped,
                    'part':   part,
                }
                
                ## adding the season section if it 
                ## would be found
                if season:
                    result.update({'season':season})
                return result
            return None
        except Exception as inst:
              ros_node.ParseException(inst)

    def get_single_season(self, pattern, mopped):
        try:
            
            has_info = re.search(pattern, mopped, re.I)
            if has_info:
                rospy.logdebug('  Looking for single season')
                start,end = has_info.span()
                section   = mopped[start:end].strip()
                mopped    = mopped[:start]+ mopped[end:]
                mopped    = mopped.strip()
                
                item = {
                    'title':  mopped
                }
                if 'E' in section:
                   item.update({'episode': section}) 
                elif 'S' in section:
                   item.update({'season': section}) 
                else:
                    rospy.logwarn("Found invalid season or episode")
                return  item
            return None
        except Exception as inst:
              ros_node.ParseException(inst)

    def get_single_date(self, pattern, mopped):
        try:
            
            has_info = re.search(pattern, mopped, re.I)
            if has_info:
                rospy.logdebug('  Looking for single date')
                start,end = has_info.span()
                section   = mopped[start:end]
                mopped    = mopped[:start]+ mopped[end:]
                mopped    = mopped.strip(', ')
                
                return  {
                    'title': mopped,
                    'date':  section
                }
            return None
        except Exception as inst:
              ros_node.ParseException(inst)

    def get_single_series(self, pattern, mopped):
        try:
            
            has_info = re.search(pattern, mopped, re.I)
            if has_info:
                rospy.logdebug('  Looking for single series')
                start,end= has_info.span()
                section  = mopped[start:end].strip()
                mopped   = mopped[:start] + mopped[end:]
                mopped   = mopped.strip(', ')
                
                return  {
                    'title':  mopped,
                    'series': section
                }
            return None
        except Exception as inst:
              ros_node.ParseException(inst)

    def get_single_episode(self, pattern, mopped):
        try:
            has_info = re.search(pattern, mopped, re.I)
            if has_info:
                rospy.logdebug('  Looking for single episode')
                start,end= has_info.span()
                section  = mopped[start:end]
                mopped   = mopped[:start] + mopped[end:]
                mopped   = mopped.strip(', ')
                
                return  {
                    'title':  mopped,
                    'episode':section
                }
            return None
        except Exception as inst:
              ros_node.ParseException(inst)

    def run(self, mopped, info):
        try:
            #print "=== === mopped:",mopped
            tags_keys = self.tags.keys()
            for idx in tags_keys:
                filters = self.tags[idx]
                for filter in filters:
                    
                    ## applying filter
                    filtered_info = self.read[filter](mopped)
                    #print "[",filter,"] === title:",mopped
                    if filtered_info:
                        #print "[",filter,"] === filtered_info:",filtered_info
                        mopped = filtered_info['title']
                        
                        ## initialising result item
                        if not info:
                            info = {}
                            #print "[",filter,"] === NEW info:",info 
                        
                        ## updating result into result item
                        info.update(filtered_info)
                        rospy.logdebug('    Updated torrent information with [%s] filter'%filter)
                    #print "[",filter,"] === info:",info 
                    #print ""
            #print "=== info:"
            #pprint(info)
            #print "=== ----:"
        except Exception as inst:
            ros_node.ParseException(inst)
        finally:
            return info

## TODO Make tests
def parse(title):
    p = TitleInfoParser()
    info = None
    info = p.run(title, info)
    pprint(info)

if __name__ == '__main__':
    logging.getLogger('imdbpie').setLevel(logging.getLevelName('DEBUG'))
    usage       = "usage: %prog option1=string option2=bool"
    parser      = OptionParser(usage=usage)
    parser.add_option('--debug', '-d',
                action='store_true',
                default=False,
                help='Provide debug level')
    parser.add_option('--parse', '-p',
                type="string",
                action='store',
                default=None,
                help='Parsing sentence')
    (options, args) = parser.parse_args()
    if options.parse is not None:
        parse(options.parse)

'''
Test strings for season_part:
https://regex101.com/r/EwWpuK/1

The Plot Against America Il Complotto Contro L America S01E01 Parte 1
'''

'''
Test strings for single_season:
https://regex101.com/r/hc5uyp/3/

Breaking Bad S01-S05
Title is Nothing S01E10
'''

'''
Test strings for [parts]:
The.Curious.Mind.with.Nigel.Latta.Series.1.4of4.What.is.Your.Brain.1080p.x264.AAC.MVGroup.org.mp4
'''
'''
Test strings for [single_series]:
    Connected The Hidden Science Of Everything Series 1 Poop
    Just another title Series 1
    Immigration Nation Series 1 Power of the Vote
    Into the Wild Colombia Series  Finding A Family
    Dino Hunters Series Dinosaur Cowboys
    Dino Hunters Series Shots Fired
    Drinks Crime and Prohibition Series Flappers and Bootleggers
'''
        
'''
Test strings for [episode_info]:
The Last Kingdom S03 E01-10
ER.S03E01.DVDRip.Xvid.avi
louie.101.dvdrip.xvid-ositv.avi
S5E13 Atlantis SquarePantis.m4v
S2E05 Christmas Who.m4v
Weeds - S03E15 - Go.mkv
Weeds.S06E01.HDTV.XviD-FQM.avi
The.Newsroom.2012.S01E01.480p.HDTV.x264-mSD.mkv
The.Wire.S01E01.The.Target.mp4
The.Wire.S04E01.mp4
top.gear.s17e03.hdtv.xvid-river.avi
Line Of Duty s01e01.thebox.hannibal.avi
top.gear.s16e00.hdtv.xvid-fov.avi
Dinosaurs.S03E01.Nature.Calls.DVDRip.XviD-DIMENSION.avi
Dinosaurs.S04E01.Monster.Under.the.Bed.DVDRip.XviD-DIMENSION.avi
The.Joy.Of.Painting.S01E01.A.Walk.In.The.Woods.DVDRip.x264.mkv
Bob Ross - The Joy of Painting - S31-01 - Reflections of Calm.avi
Stargate.Universe.S02E12.HDTV.XviD-ASAP.avi
my_name_is_earl.2x01.very_bad_things.dvdrip_xvid-fov.avi
1x02 Inappropriate.avi
Babylon 5 S01E01 - Midnight on the Firing Line.avi
aaf-midsomer.murders.s13e01.dvdrip.avi
Midsomer.Murders.S14E01.720p.HDTV.x264-BiA.mkv
babylon5.s02e05.dvdrip.xvid-sfm.avi
babylon.5.s03e19.dvdrip.xvid-sfm.avi
Babylon.5.S04E08.DVDRip.XviD-SFM.avi
Babylon.5.S05E06.DVDRip.XviD-SFM.avi
Torchwood.S04E01.HDTV.XviD-LOL.avi
Dallas.S01.E02.The.Lesson-EiNSTEiN.avi
breaking.bad.s01e01.dvdrip.xvid-orpheus.avi
project.runway.all.sstars.s01e07.720p.hdtv.x264-2hd.mkv
Work.Of.Art.The.Next.Great.Artist.S02E08.Sell.Out.PROPER.480p.HDTV.x264-mSD.mkv
Law & Order - 13x17 - Genius [HDTV-FQM].avi
Law.and.Order.S10E01.Gunshow.HDTV.XviD-SAiNTS.avi
law.and.order.svu.1317.hdtv-lol.mp4
Wings.S01E04.Return.to.Nantucket.Pt.II.DVDRip.XviD-DIMENSION.avi
True.Blood.S05E07.480p.HDTV.x264-mSD.mkv
avcdvd-strike.back.s01e03-e04.bdrip.xvid.avi
JAG.S09E01.DVDRip.XviD-P0W4DVD\JAG.S09E01.DVDRip.XviD-P0W4DVD.avi
terra.nova.s01e01.hdtv.xvid-fqm.avi
top_gear.18x03.hdtv_xvid-fov.avi
Weeds.S01E01.DVDRip.XviD-TOPAZ.avi
undercover.boss.us.s01e07.pdtv.xvid-2hd.avi
undercover.boss.us.s02e01.hdtv.xvid-2hd.avi
S02E06 The Dog Father.avi
aaf-wilfred.s02e01.kiss.me.kat.avi
femme.fatales.s01e01.hdtv.xvid-sys.avi
eagleheart.s01e09.hdtv.xvid-fqm.avi
game.of.thrones.s01e01.hdtv.xvid-fever.avi
ER.S01E01-02.DVDRip.XviD.avi
Weeds.S02E09.DVDRip.XviD-TOPAZ.avi
ER.S02E01.DVDRip.Xvid.avi
ER.S04-01 (East Coast).avi
ER.S05E01.DVDRip.Xvid.avi
tpz-30rock101.avi
MASH - 01x24 - Showtime.mkv
Louie.S02E01.BDRip.XviD-DEiMOS.avi
Psych.S01E01.DVDRip.XviD-TOPAZ.avi
Psych.S06E02.HDTV.XviD-P0W4.avi
Weeds.S03E01.Mobile.Device.Blu-ray.DTS.H.264-BTN.m4v
weeds.s04e10.dvdrip.xvid-phase.avi
Up.All.Night.2011.S01E07.HDTV.XviD-LOL.avi
So.You.Think.You.Can.Dance.S04E01.HDTV.XviD-ORENJi.avi
the_fades.1x01.hdtv_xvid-fov.avi
Sponge Bob [season 06][episod 101a] - House Fancy.avi
SpongeBob SquarePants - s1e00x - Intro [Fester1500].avi
Sponge Bob [season 04][episod 11] - Dunces and Dragons.avi
Sponge Bob [season 03][episod 16] - The Lost episode.avi
Weeds.S05E01.720p.HDTV.X264-DIMENSION\Weeds.S05E01.720p.HDTV.X264-DIMENSION.mkv
weeds.s07e01.hdtv.xvid-asap.avi
The.Wire.S02E01.Ebb.Tide.mp4
The.Wire.S03E01.Time.After.Time.mp4
The.Wire.S05E01.mp4
24.S04E01.WS.DVDRip.XviD-MEDiEVAL.avi
Psych.2x01.American_Duos.DVDRip_XviD-FoV.avi
24.s05e00.special.intro.ws.dsr.xvid-crntv.avi
homicide.life.on.the.street.319.dvdrip-lol.avi
homicide.s06e21-med.avi
gilmore_girls.1x03.kill_me_now.dvdrip_xvid-fov.avi
Holmes on Homes - S07E00 - Lien On Me.avi
dance.moms.s01e06.dying.to.dance.hdtv.xvid-crimson.avi
Greys.Anatomy.S04E01.A.Change.Is.Gonna.Come.DVDrip.XviD-ORPHEUS.avi
Wilfred - 1. There Is A Dog [Xvid].avi
greys.anatomy.s05e01.dvdrip.xvid-reward.avi
Line.Of.Duty.1x02.480p.HDTV.x264-mSD.mkv
S03E18.This Guy Smith.avi
retired.at.35.s01e02.dsr.xvid-fqm.avi
psych.s03e01.dvdrip.xvid-reward.avi
Seven_Days.S01E12.DigiRip.XviD-BamVCD.avi
the.walking.dead.s02e10.hdtv.xvid-2hd.avi
24.s06e01.dvdrip.xvid-medieval.avi
Seven_Days.S02E21.DigiRip.XviD-BamVCD.avi
survivorman.302.hdtv.xvid-sys.avi
Survivorman.S02E03.DVDRip.XviD-FFNDVD\survivorman.s02e03.dvdrip.xvid-ffndvd.avi
Survivorman 101 Canadian Boreal Forest(Dvd).avi
Bones.S01E01.DVDRip.XviD-TOPAZ.avi
Weed.Wars.S01E01.Worlds.Largest.Medical.Marijuana.Dispensary.HDTV.XviD-
Episode 01 Icelandc Volcano.avi
The Darling Buds of May -107Spec- Christmas is Coming.avi
greys.anatomy.s06e01.dvdrip.xvid-reward.avi
hit.and.miss.s01e01-tastetv.avi
swamp.loggers.s03e01.crisis.at.the.mill.hdtv.xvid-momentum.avi
nurse.jackie.s01e01.dvdrip.xvid-reward.avi
top.gear.s14e07.hdtv.xvid-bia.avi
las.vegas.s02e01.ws.dvdrip.xvid-medieval.avi
las.vegas.s05e01.dvdrip.xvid-studio.avi
Cheers.S09E01.Love.Is.A.Really.Really.Perfectly.Okay.Thing.DVDRip.XviD-SAiNTS\Cheers.S09E01.Love.Is.A.Really.Really.Perfectly.Okay.Thing.DVDRip.Xvi
S01E01 - Love is All Around.avi
My_Name_Is_Earl.1x01.Pilot.DVDRip_XviD-FoV\Sample\my_name_is_earl.1x01.pilot.dvdrip_xvid-sample-fov.avi
Wings.S04E01.Lifeboat.DVDRip.XviD-SAiNTS.avi
Wings.S03E01.The.Naked.Truth.DVDRip.XviD-SAiNTS.avi
'''
