import rospy
import pyesiaf
from esiaf_ros.msg import RecordingTimeStamps
import soundfile


def calc_frametime(file, framesize):
    sr = soundfile.info(file).samplerate
    return rospy.Duration.from_sec(framesize/sr)


class WavPlayer():

    def __init__(self, esiaf_handler, file, realtime_factor, topic, framesize):
        self.esiaf_handler = esiaf_handler
        self.file = file
        self.realtime_factor = realtime_factor
        self.topic = topic
        self.start_time = None
        self.framesize = framesize
        self.frametime = calc_frametime(file, framesize)

    def play(self):
        for block in soundfile.blocks(self.file, blocksize=self.framesize):

            if not self.start_time:
                self.start_time = rospy.Time.now()
            end_time = self.start_time + self.frametime
            timestamps = RecordingTimeStamps()
            timestamps.start = self.start_time
            timestamps.finish = end_time

            rospy.sleep(end_time - rospy.Time.now())

            self.esiaf_handler.publish(self.topic, block, timestamps)
            self.start_time = end_time