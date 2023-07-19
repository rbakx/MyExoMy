import subprocess
import re
import time

def get_wifi_list():
    command = "nmcli -t -f IN-USE,SSID,BSSID,SIGNAL dev wifi list"
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
        in_use, ssid, bssid_with_backslashes, signal = re.split(r'(?<!\\):', line)
        bssid = bssid_with_backslashes.replace("\\","")
        signal_strength = int(signal) # Signal strength = 0..100%
        if in_use == "*":
            current_access_point = ssid
            current_signal_strength = signal_strength
        if ssid in access_points_to_compare:
            print (line)
            if signal_strength > largest_signal:
                largest_signal = signal_strength
                best_access_point = ssid
    # Only consider an access point as the best if the signal strength is significantly higher
    # than the current signal strength. This to prevent frequent switching between access points.
    if largest_signal < current_signal_strength + 10:
        best_access_point = current_access_point
    return best_access_point,current_access_point

def connect_to_access_point(access_point):
    if access_point:
        try:
            subprocess.check_call(f"nmcli d wifi connect {access_point}", shell=True)
            print(f"Connected to {access_point}.")
        except subprocess.CalledProcessError:
            print("Error connecting to the specified access point.")
    else:
        print("None of the specified access points found.")

if __name__ == "__main__":
    access_points_to_compare = ["wifiwifiwifi_guest", "wifig", "123connect"]  # Replace with the access points you want to compare
    while True:
        wifi_list = get_wifi_list()
        if wifi_list:
            best_access_point,current_access_point = find_access_point_with_largest_signal(wifi_list, access_points_to_compare)
            print(f"best access point: {best_access_point}, current access point: {current_access_point}")
            if best_access_point != current_access_point:
                print(f"going to switch from {current_access_point} to {best_access_point}")
                connect_to_access_point(best_access_point)
            else:
                print(f"current access point {current_access_point} is already the best access point, no switch")

        else:
            print("No Wi-Fi networks found.")
        time.sleep(10)
