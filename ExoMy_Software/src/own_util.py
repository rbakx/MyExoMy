#!/usr/bin/python
import subprocess
import re


# runShellCommandWait(cmd) will block until 'cmd' is finished.
# This because the communicate() method is used to interact with the process through the redirected pipes.
def runShellCommandWait(cmd):
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True).communicate()[0]

# runShellCommandNowait(cmd) will not block because the communicate() method is not used.
# Therefore do not use 'stdout=subprocess.PIPE, stderr=subprocess.STDOUT' here as these pipes will not be read or written
# which means the process can block after a while. This happens for example when running mplayer with runShellCommandNowait().
# Therefore redirection of stdout and stderr to /dev/null is used.
def runShellCommandNowait(cmd):
    subprocess.Popen(cmd + '>/dev/null 2>&1', shell=True)


# The same functions as above but now to execute commands from a Docker container on the Raspberry Pi host.
# This is done through SSH. SSH keys for identification must be installed for this to work.
# On Docker for Linux, the IP address of the gateway between the Docker host and the bridge network is 172.17.0.1 if you are using default networking.
# See also https://upcloud.com/community/tutorials/use-ssh-keys-authentication/.
def HostRunShellCommandWait(cmd):
    return subprocess.Popen("ssh pi@172.17.0.1 " + "\"" + cmd + "\"", stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True).communicate()[0]


def HostRunShellCommandNowait(cmd):
    subprocess.Popen("ssh pi@172.17.0.1 " + "\"" + cmd + "\"" + ">/dev/null 2>&1", shell=True)

def HostStartVideoStream():
    # Run on Raspberry Pi host, so through SSH.
    # On Docker for Linux, the IP address of the gateway between the Docker host and the bridge network is 172.17.0.1 if you are using default networking.
    HostRunShellCommandNowait("cd /home/pi/ExoMy_Software/webrtc-web/work && node index.js")
    HostRunShellCommandNowait( "export DISPLAY=:0 nohup; xhost si:localuser:root; sudo chromium-browser --no-sandbox --ignore-certificate-errors --disable-restore-session-state --start-fullscreen https://localhost:8080")

    
def HostStopVideoStream():
    # Run on Raspberry Pi host, so through SSH.
    # On Docker for Linux, the IP address of the gateway between the Docker host and the bridge network is 172.17.0.1 if you are using default networking.
    HostRunShellCommandWait("sudo pkill -f \"index.js\"")
    HostRunShellCommandWait("sudo pkill -f \"chromium\"")

def HostShutdown():
    # Run on Raspberry Pi host, so through SSH.
    # On Docker for Linux, the IP address of the gateway between the Docker host and the bridge network is 172.17.0.1 if you are using default networking.
    HostRunShellCommandNowait("sudo shutdown -P now")


def HostGetWifiStatus():
    # Run on Raspberry Pi host, so through SSH.
    stdOutAndErr = HostRunShellCommandWait("/sbin/iwconfig")
    # Use DOTALL (so '.' will also match a newline character) because stdOutAndErr can be multiline.
    expr = re.compile('.*ESSID:"(.*)".*?Signal level=(.*dBm)', re.DOTALL)
    m = expr.match(stdOutAndErr)
    if m is not None:  # m will be not None only when both capture groups are valid.
        wifiStatus =  m.group(1) + ', ' + m.group(2)
    else:
        wifiStatus = 'wifi unknown'
    return wifiStatus
