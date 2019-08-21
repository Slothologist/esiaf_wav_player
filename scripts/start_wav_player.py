#!/usr/bin/env python

from esiaf_wav_player.wav_player import WavPlayer
import pyesiaf
import rospy
from std_msgs.msg import String

# config
import yaml
import sys

# file handling
from os import listdir
from os.path import isfile, join

# audio file handling
import soundfile

# sleeping
import time

# threading imports
import threading

def esiaf_format_from_soundfile_info(sf_info):
    esiaf_format = pyesiaf.EsiafAudioFormat()
    # samplerate
    if sf_info.samplerate == 8000:
        esiaf_format.rate = pyesiaf.Rate.RATE_8000
    if sf_info.samplerate == 16000:
        esiaf_format.rate = pyesiaf.Rate.RATE_16000
    if sf_info.samplerate == 32000:
        esiaf_format.rate = pyesiaf.Rate.RATE_32000
    if sf_info.samplerate == 44100:
        esiaf_format.rate = pyesiaf.Rate.RATE_44100
    if sf_info.samplerate == 48000:
        esiaf_format.rate = pyesiaf.Rate.RATE_48000
    if sf_info.samplerate == 96000:
        esiaf_format.rate = pyesiaf.Rate.RATE_96000

    # bitrate
    esiaf_format.bitrate = pyesiaf.Bitrate.BIT_FLOAT_64

    # channel
    esiaf_format.channels = sf_info.channels

    # endian
    esiaf_format.endian = pyesiaf.Endian.BigEndian if sf_info.endian == 'BIG' else pyesiaf.Endian.LittleEndian
    return esiaf_format

# initialize rosnode
rospy.init_node('esiaf_wav_player')
pyesiaf.roscpp_init('esiaf_wav_player', [])

# read config
rospy.loginfo('Loading config...')
argv = sys.argv
if len(argv) < 2:
    rospy.logerr('Need path to configfile as first parameter!')
    exit('1')
path_to_config = argv[1]
data = yaml.safe_load(open(path_to_config))

rospy.loginfo('Acquiring wav files...')

files = []

if data['file'] == "*":
    files = [join(data['file_directory'], f) for f in listdir(data['file_directory']) if isfile(join(data['file_directory'], f)) and '.wav' in f]
else:
    files.append(join(data['file_directory'], data['file']))

rospy.loginfo('Playback of files: ' + str([file.split('/')[-1] for file in files]))

rospy.loginfo('Acquiring wav format, assuming every file has the same format...')


info = soundfile.info(files[0])


rospy.loginfo('Creating esiaf handler...')
handler = pyesiaf.Esiaf_Handler("wav_player", pyesiaf.NodeDesignation.Other, sys.argv)

rospy.loginfo('Setting up esiaf...')
esiaf_format = esiaf_format_from_soundfile_info(info)
esiaf_audio_info = pyesiaf.EsiafAudioTopicInfo()
esiaf_audio_info.topic = data['esiaf_output_topic']
esiaf_audio_info.allowedFormat = esiaf_format

rospy.loginfo('adding output...')
handler.add_output_topic(esiaf_audio_info)
handler.start_esiaf()

finished_pub = rospy.Publisher('/esiaf/wav_player/finished_playing', String, queue_size= 100)


# create the wav players
def player_loop():
    time.sleep(2)
    for filename in files:
        rospy.loginfo('Creating Wav player for ' + filename + ' ...')
        player = WavPlayer(handler,
                           filename,
                           data['playback_speed'],
                           data['esiaf_output_topic'],
                           data['default_frame_size']
                           )
        player.play()
        rospy.loginfo('playing finished!')
        #time.sleep(data['time_between_files_ms']/1000)
        fin = String(filename)
        finished_pub.publish(fin)
    pub = rospy.Publisher('/esiaf/wav_player/shutdown', String, queue_size=1)
    pub.publish(String('done'))
    time.sleep(2)
    rospy.signal_shutdown('successfully finished')


t = threading.Thread(target=player_loop)
t.start()


rospy.loginfo('Wav player ready!')
rospy.spin()

handler.quit_esiaf()
