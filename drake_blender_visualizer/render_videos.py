import os

for k in range(3):
    rotation_str = ""
    if k == 0:
        rotation_str = '-filter:v transpose=2 -metadata:s:v rotate=""'
    elif k == 1:
        rotation_str = '-filter:v transpose=1 -metadata:s:v rotate=""'
    elif k == 2:
        rotation_str = '-filter:v transpose=1 -metadata:s:v rotate=""'
    os.system("avconv -y -r 120 -i /tmp/manipulation_station_ycb/%02d_%%08d.jpg /tmp/manipulation_station_ycb/manipulation_station_ycb_render_realtime_%02d.mp4" % (k, k))
    os.system("avconv -y -i /tmp/manipulation_station_ycb/manipulation_station_ycb_render_realtime_%02d.mp4 %s manipulation_station_ycb_render_realtime_%02d.mp4" % (k, rotation_str, k))


#for k in range(25):
#    os.system("ffmpeg -y -r 30 -i /tmp/ycb_scene_%03d/00_%%8d.jpg /tmp/ycb_scene_%03d/render_halftime.mp4" % (k, k))
#    os.system("ffmpeg -y -r 60 -i /tmp/ycb_scene_%03d/00_%%8d.jpg /tmp/ycb_scene_%03d/render_realtime.mp4" % (k, k))
