import picamera
import time
import os

dir_count = 0
max_count = 0
for file_name in os.listdir('.'):
    if file_name.find("capture_") >= 0:
        actual_count = int(file_name.replace('capture_',''))
        if actual_count > max_count:
            max_count = actual_count
dir = "capture_%03d"%(max_count+1)
os.mkdir(dir)


with picamera.PiCamera() as camera:
    camera.resolution = (3280,2464)
    camera.iso = 400
    for i in range(210):
	print("photo %s taken"%i)
        camera.capture(dir+"/image%s.jpg"%i)
	time.sleep(2)
