#!/usb/bin/env python

import sys, os
import picamera

#tell python where to find mavlink s we can import it
from pymavlink import mavutil
from geopy import distance

#geotag imports
import time
import pyexiv2
import fractions
from PIL import Image
from PIL.ExifTags import TAGS

DEFAULT_MIN_DIST_METERS = 2

class LIA_GeoPhoto:
    def __init__(self):
        self.file_name_prefix = 'img'
        self.counter = 0
        self.set_flight_number()
        self.camera = picamera.PiCamera()
        self.camera.resolution = (3280,2464)
        self.camera.iso = 400


    def set_flight_number(self):
        dir_count = 0
        max_count = 0
        for file_name in os.listdir('.'):
            if file_name.find("flight") >= 0:
                actual_count = int(file_name.replace('flight',''))
                if actual_count > max_count:
                    max_count = actual_count
        self.flight_dir = "flight%03d"%(max_count+1)
        os.mkdir(self.flight_dir)

    def to_deg(self, value,loc):
        if value < 0:
            loc_value = loc[0]
        elif value > 0:
            loc_value = loc[1]
        else:
            loc_value = ""
        abs_value = abs(value)
        deg = int(abs_value)
        t1 = (abs_value-deg)*60
        min = int(t1)
        sec = round((t1 - min)*60 , 5)
        return (deg,min, sec,loc_value)

    def set_gps_location(self,file_name,tags):
        lat = tags.lat/10000000.0
        lng = tags.lng/10000000.0

        lat_deg = self.to_deg(lat, ["S", "N"])
        lng_deg = self.to_deg(lng, ["W", "E"])

        #conver decimal coordinates into degrees, minutes and seconds

        exiv_lat = (pyexiv2.Rational(lat_deg[0]*60+lat_deg[1],60),pyexiv2.Rational(lat_deg[2]*100,6000),pyexiv2.Rational(0,1))
        exiv_lng = (pyexiv2.Rational(lng_deg[0]*60+lng_deg[1],60),pyexiv2.Rational(lng_deg[2]*100,6000),pyexiv2.Rational(0,1))

        exiv_image = pyexiv2.ImageMetadata(file_name)
        exiv_image.read()
        exif_keys = exiv_image.exif_keys
        exiv_altitude = pyexiv2.Rational(tags.altitude,1)
        print(exiv_lat,exiv_lng,lat_deg[3],lng_deg[3],exiv_altitude)
        if tags.altitude > 0:
            exiv_image["Exif.GPSInfo.GPSLatitude"] = exiv_lat
            exiv_image["Exif.GPSInfo.GPSLatitudeRef"] = lat_deg[3]
            exiv_image["Exif.GPSInfo.GPSLongitude"] = exiv_lng
            exiv_image["Exif.GPSInfo.GPSLongitudeRef"] = lng_deg[3]
            exiv_image["Exif.GPSInfo.GPSAltitude"] = exiv_altitude
            exiv_image["Exif.GPSInfo.GPSAltitudeRef"] = '0'
            exiv_image["Exif.Image.GPSTag"] = 654
            exiv_image["Exif.GPSInfo.GPSMapDatum"] = "WGS-84"
            exiv_image["Exif.GPSInfo.GPSVersionID"] = '2 0 0 0'

            exiv_image.write()

    def shot_picture(self):
        self.file_name = self.flight_dir + os.sep + "%s%04d.jpg"%(self.file_name_prefix,self.counter)
        print ("Capture %s\n"%(self.file_name))
        self.counter += 1
	file.close()


    def geotag(self,tags):
        lat = tags.lat/10000000.0
        lon = tags.lng/10000000.0
        self.set_gps_location(self.file_name,float(lat),float(lon))


    def save(self,tags):
        self.shot_picture()
        self.set_gps_location(self.file_name,tags)


def read_loop(m):
    plat = 0
    plon = 0
    photo_counter = 0
    geophoto = LIA_GeoPhoto()
    while(True):
        #grab a mavlink message
        msg = m.recv_match(blocking=False)
        if msg:
            #handlethe mesage based on its type
            msg_type = msg.get_type()
            if msg_type == "AHRS3":
                lat = int(msg.lat)/10000000.0
                lon = int(msg.lng)/10000000.0
                alt = msg.altitude
                roll = msg.roll
                pitch = msg.pitch
                yaw = msg.yaw
                if plat == 0 and lat != 0:
                    plat = lat
                    plon = lon

                elif (plat + lat != 0):
                    ant = (plat,plon)
                    actual = (lat,lon)
                    dist = distance.distance(ant,actual).m
                    print (dist)
                    if (dist > DEFAULT_MIN_DIST_METERS):
                        print("Shot Picture")
                        geophoto.save(msg)
                        plat = lat
                        plon = lon

def main():

    ldevice = "/dev/ttyAMA0"
    lbaudrate = 57600
    lrate = 4


    #create a mavlink serial instance
    master = mavutil.mavlink_connection(ldevice, baud=lbaudrate)

    #wait for the heartbeat message to find system ID

    print("wainting Heartbeat")
    master.wait_heartbeat()
    print("heartbeat_recived")
    #request data to be sent at the given rate
    master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL,lrate,1)

    #enter the data loop 
    read_loop(master)



if __name__ == '__main__':
    main()
