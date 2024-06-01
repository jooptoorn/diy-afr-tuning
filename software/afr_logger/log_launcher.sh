#!/bin/sh
# log_launcher.sh
# launches the disklogger that reads AFR values from a USB-connected afr-logger and stores them to disklogger

cd /home/joop/
sudo python log_afr_to_disk.py
