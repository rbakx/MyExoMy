#!/usr/bin/env python
from subprocess import call
import os
import time

exomy_shutdown_file = '/home/pi/ExoMy_Software/exomy_shutdown.txt'
while True:
  if os.path.exists(exomy_shutdown_file):
    os.remove(exomy_shutdown_file)
    call("sudo shutdown -P now", shell=True)
  time.sleep(10)
