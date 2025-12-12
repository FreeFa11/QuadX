# This scripts copies the Webpage files into the Firmware

import os
import shutil



Files = ["Controller.html", "Calibration.html", "Connection.html", "calibration-icon.svg", "connection-icon.svg", "controller-icon.svg", "favicon.ico"]



source_dir = os.path.dirname(os.path.realpath(__file__))
destination_dir = os.path.join(os.path.dirname(source_dir), "Firmware/data")

for File in Files:
    shutil.copy(os.path.join(source_dir, File), destination_dir)