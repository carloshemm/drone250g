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
    #camera.resolution = (3280,2464) #Para camera versão 2
    camera.resolution = (2592,1944) #Para camera versão 1.2
    camera.iso = 400
    for i in range(210):
	print(f"photo {string(i)} taken")
        camera.capture(dir+f"/image{string(i)}.jpg")
	time.sleep(2)
