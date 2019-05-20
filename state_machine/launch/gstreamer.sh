#!/usr/bin/env bash
gst-launch-1.0 -v v4l2src device="/dev/video1" ! video/x-raw,width=1920,height=1080,framerate=5/1 ! x264enc ! rtph264pay ! udpsink host=192.168.0.228 port=5001 &
gst-launch-1.0 -v v4l2src device="/dev/video2" ! video/x-raw,width=1920,height=1080,framerate=5/1 ! x264enc ! rtph264pay ! udpsink host=192.168.0.228 port=5002 &
gst-launch-1.0 -v v4l2src device="/dev/video3" ! video/x-raw,width=1920,height=1080,framerate=5/1 ! x264enc ! rtph264pay ! udpsink host=192.168.0.228 port=5003
