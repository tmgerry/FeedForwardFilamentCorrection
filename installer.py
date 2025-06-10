# To Run this scrip execute the following command-
#
## cd && cd CSU-in-situ/ && python3 klippy/installer.py && cd
#
# --- To restart Klipper ---
#
## sudo service klipper start
#
# --- To do both ---
#
## cd && cd CSU-in-situ/ && python3 klippy/installer.py && cd && sudo service klipper restart


import shutil
import os
home_dir = os.path.expanduser("~")
print(home_dir)
print("Updating Git Repo")
os.system('git pull')

print("Syncing Modules")

# tmag sensor
dst = f"{home_dir}/klipper/klippy/extras/tmag5273_filament_width_sensor.py"
src = "klippy/extras/tmag5273_filament_width_sensor.py"
shutil.copyfile(src, dst)
print(".")

# tmag5273 temp sensor
dst = f"{home_dir}/klipper/klippy/extras/tmag5273_temp.py"
src = "klippy/extras/tmag5273_temp.py"
shutil.copyfile(src, dst)
print("..")

# temperature sensors.cfg
dst = f"{home_dir}/klipper/klippy/extras/temperature_sensors.cfg"
src = "klippy/extras/temperature_sensors.cfg"
shutil.copyfile(src, dst)
print("...")

# statistics
dst = f"{home_dir}/klipper/klippy/extras/statistics.py"
src = "klippy/extras/statistics.py"
shutil.copyfile(src, dst)
print("....")

# gcode_move
dst = f"{home_dir}/klipper/klippy/extras/gcode_move.py"
src = "klippy/extras/gcode_move.py"
shutil.copyfile(src, dst)
print(".....")

# motion_report
dst = f"{home_dir}/klipper/klippy/extras/motion_report.py"
src = "klippy/extras/motion_report.py"
shutil.copyfile(src, dst)
print("......")

# Display Mods For PRUSA i3 MK3S+
dst = f"{home_dir}/klipper/klippy/extras/display/display.cfg"
src = "klippy/extras/display.cfg"
shutil.copyfile(src, dst)
print(".......")


print("Completed, custom Klipper Modules up to date!")