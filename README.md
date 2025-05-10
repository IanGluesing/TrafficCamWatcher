# TrafficCamWatcher


cam_stream.cpp -o cam_stream `pkg-config --cflags --libs opencv4` -std=c++11

ffmpeg -ss 00:00:00.000 -i example_recording.mov -pix_fmt rgb24 -r 10 -s 320x240 -t 00:01:00.000 output.gif