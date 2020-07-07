 #ssh into your raspberry pi
 #create folder bbt $ cd bbt
 #make the script executable $ chmod 755 launcher.sh
 #execute $ sh launcher.sh
 #create folder for log $ cd && mkdir logs
 #make execute the script at stratup $ sudo crontab -e
 #insert in the last line $ @reboot sh /home/pi/bbt/launcher.sh >/home/pi/logs/cronlog 2>&1
 #test the script $ sudo reboot
