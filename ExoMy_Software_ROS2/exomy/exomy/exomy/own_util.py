#!/usr/bin/python
import subprocess
import re


# RunShellCommandWait(cmd) will block until 'cmd' is finished.
# This because the communicate() method is used to interact with the process through the redirected pipes.
def RunShellCommandWait(cmd):
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True).communicate()[0]

# RunShellCommandNowait(cmd) will not block because the communicate() method is not used.
# Therefore do not use 'stdout=subprocess.PIPE, stderr=subprocess.STDOUT' here as these pipes will not be read or written
# which means the process can block after a while. This happens for example when running mplayer with runShellCommandNowait().
# Therefore redirection of stdout and stderr to /dev/null is used.
def RunShellCommandNowait(cmd):
    subprocess.Popen(cmd + '>/dev/null 2>&1', shell=True)

def StartVideoStream():
    # Start the node.js signalling server.
    RunShellCommandNowait("cd /home/ubuntu/exomy/webrtc-web && node index.js")
    # The Chromium browser should be started with the command below but unfortunately it crashes with 'Trace/breakpoint trap'.
    # Hopefully this will be resolved in a future Chromium version.
    RunShellCommandNowait("firefox https://localhost:8080")

def StopVideoStream():
    # Run on Raspberry Pi host, so through SSH.
    # On Docker for Linux, the IP address of the gateway between the Docker host and the bridge network is 172.17.0.1 if you are using default networking.
    RunShellCommandWait("sudo pkill -f \"index.js\"")
    RunShellCommandWait("sudo pkill -f \"firefox\"")

def Shutdown():
    # Run on Raspberry Pi host, so through SSH.
    # On Docker for Linux, the IP address of the gateway between the Docker host and the bridge network is 172.17.0.1 if you are using default networking.
    RunShellCommandNowait("sudo shutdown -P now")


def GetWifiStatus():
    # Run on Raspberry Pi host, so through SSH.
    stdOutAndErr = RunShellCommandWait("/sbin/iwconfig")
    # Use DOTALL (so '.' will also match a newline character) because stdOutAndErr can be multiline.
    expr = re.compile('.*ESSID:"(.*)".*?Signal level=(.*dBm)', re.DOTALL)
    m = expr.match(stdOutAndErr.decode('utf-8'))
    if m is not None:  # m will be not None only when both capture groups are valid.
        wifiStatus =  m.group(1) + ', ' + m.group(2)
    else:
        wifiStatus = 'wifi unknown'
    return wifiStatus
