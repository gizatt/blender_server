import os

for k in range(25):
    os.system("ffmpeg -y -r 30 -i /tmp/ycb_scene_%03d/00_%%8d.jpg /tmp/ycb_scene_%03d/render_halftime.mp4" % (k, k))
    os.system("ffmpeg -y -r 60 -i /tmp/ycb_scene_%03d/00_%%8d.jpg /tmp/ycb_scene_%03d/render_realtime.mp4" % (k, k))
