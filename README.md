# esiaf_wav_player
A node for the esiaf framework that feeds the pipeline with audio from a wav file.
For more information about esiaf visit [its repository](https://github.com/Slothologist/esiaf_ros).

## Features of this player

- can play basically any soundfile available through libsoundfile (including but not limited to .wav, .au, .aiff, .flac)
- can play either one file or all files in a given directory
- can take pause between playing several files
- can play files faster or slower without interfering with the format (this means you can gain faster-than-realtime speeds for recognition, provided your components can keep up; work in progress)


## Dependencies
- [esiaf_ros](https://github.com/Slothologist/esiaf_ros) and its dependencies (mainly ROS)
- libsoundfile (libsndfile1 in ubuntu 16.04/ 18.04)
