# import subprocess

# device_list = subprocess.run(["./find_devpath.bash"], stdout=subprocess.PIPE, text=True).stdout.splitlines()

# devpath_list = ["", "", ""]

# # Add device paths to devpath list
# for device in device_list:
#     splitStr = device.split(" - ")

#     if "ID0001" in splitStr[1]:
#         devpath_list[0] = splitStr[0]

#     elif "ID0002" in splitStr[1]:
#         devpath_list[1] = splitStr[0]

#     elif "ID0003" in splitStr[1]:
#         devpath_list[2] = splitStr[0]


# for devpath in devpath_list:
#     print(devpath)

import subprocess
getter_script = ""
        
device_list = subprocess.run(["ls", "-a"], stdout=subprocess.PIPE, text=True).stdout.splitlines()

for device in device_list:
    print(device)

devpath_list = ["", "", ""]