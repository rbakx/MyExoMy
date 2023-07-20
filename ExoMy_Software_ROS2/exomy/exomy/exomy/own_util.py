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
        wifiStatus = m.group(1) + ', ' + m.group(2)
    else:
        wifiStatus = 'wifi unknown'
    return wifiStatus


def get_wifi_list():
    command = "sudo nmcli -t -f IN-USE,SSID,BSSID,SIGNAL dev wifi list"
    try:
        output = subprocess.check_output(command, shell=True, text=True)
        return output.strip().splitlines()
    except subprocess.CalledProcessError:
        print("Error executing the command.")
        return []


def find_access_point_with_largest_signal(wifi_list, access_points_to_compare):
    largest_signal = -100  # Initialize with a very low value
    current_signal_strength = -100
    best_access_point = None
    current_access_point = None

    for line in wifi_list:
        in_use, ssid, bssid_with_backslashes, signal = re.split(
            r'(?<!\\):', line)
        bssid = bssid_with_backslashes.replace("\\", "")
        signal_strength = int(signal)  # Signal strength = 0..100%
        if in_use == "*":
            current_access_point = ssid
            current_signal_strength = signal_strength
        if ssid in access_points_to_compare:
            print(line)
            if signal_strength > largest_signal:
                largest_signal = signal_strength
                best_access_point = ssid
    # Adding hysteresis: only consider an access point as the best if the signal strength is significantly higher
    # than the current signal strength. This to prevent frequent switching between access points.
    # Can be replaced with '+ 10' to add hysteresis.
    if largest_signal < current_signal_strength + 0:
        best_access_point = current_access_point
    return best_access_point, current_access_point


def connect_to_access_point(access_point):
    if access_point:
        try:
            subprocess.check_call(
                f"sudo nmcli d wifi connect {access_point}", shell=True)
            print(f"Connected to {access_point}.")
        except subprocess.CalledProcessError:
            print("Error connecting to the specified access point.")
    else:
        print("None of the specified access points found.")


def CheckWifi():
    # Replace with the access points you want to compare
    access_points_to_compare = ["wifiwifiwifi_guest", "wifig", "123connect"]
    wifi_list = get_wifi_list()
    if wifi_list:
        best_access_point, current_access_point = find_access_point_with_largest_signal(
            wifi_list, access_points_to_compare)
        print(
            f"best access point: {best_access_point}, current access point: {current_access_point}")
        if True:  # Can be replaced with 'if best_access_point != current_access_point:' to only switch when the access point if different.
            print(
                f"going to switch from {current_access_point} to {best_access_point}")
            connect_to_access_point(best_access_point)
        else:
            print(
                f"current access point {current_access_point} is already the best access point, no switch")

    else:
        print("No Wi-Fi networks found.")
