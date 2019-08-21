import rospy
from esiaf_ros.msg import RecordingTimeStamps
import soundfile
import StringIO
import numpy

def msg_to_string(msg):
    buf = StringIO.StringIO()
    msg.serialize(buf)
    return buf.getvalue()


def calc_frametime(file, framesize):
    sr = soundfile.info(file).samplerate
    frac = float(framesize)/sr
    return rospy.Duration.from_sec(frac)


class WavPlayer():

    def __init__(self, esiaf_handler, file, playback_speed, topic, framesize):
        self.esiaf_handler = esiaf_handler
        self.file = file
        self.topic = topic
        self.start_time = None
        self.framesize = framesize
        self.frametime = playback_speed*calc_frametime(file, framesize)

    def play(self):
        for block in soundfile.blocks(self.file, blocksize=self.framesize):

            if not self.start_time:
                self.start_time = rospy.Time.now()
            end_time = self.start_time + self.frametime
            timestamps = RecordingTimeStamps()
            timestamps.start = self.start_time
            timestamps.finish = end_time

            rospy.sleep(end_time - rospy.Time.now())
            self.esiaf_handler.publish(self.topic,
                                       block,
                                       msg_to_string(timestamps)
                                       )
            self.start_time = end_time

        frames_per_second = 16000
        timeout_in_seconds = 2. * 400./1000.
        five_twelve_blocks = int(numpy.ceil(frames_per_second * timeout_in_seconds/512))
        filler_array = numpy.zeros((512,), numpy.int16)

        for _ in range(five_twelve_blocks):
            end_time = self.start_time + self.frametime
            timestamps = RecordingTimeStamps()
            timestamps.start = self.start_time
            timestamps.finish = end_time

            rospy.sleep(end_time - rospy.Time.now())
            self.esiaf_handler.publish(self.topic,
                                       filler_array,
                                       msg_to_string(timestamps)
                                       )
            self.start_time = end_time

