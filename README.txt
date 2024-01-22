README for the Beacon Bros’ AE 483 Final Project

This folder contains the following files:

BeaconBros Main #######################################
# ProjectDataAnalysis - self explanatory, the final .ipynb file used to produce all the data in the report.
# flight.py - the flight python files contain all of the different move commands used to produce data for the report and videos, named accordingly.
# controller_ae483.c - controller files contain both custom custom observer implementations, named accordingly.

FlightTestData #######################################
# .json files for each of the final flight tests conducted on the 3 different observer models.
# one additional file for a failed flight test of a more complex Kalman Filter model.

FlightTestVideos #######################################
# .MOV files corresponding to each flight test data file of the same name.

LabFiles #######################################
# Data collected from and analyzed for labs 3-5 and 7-9 to account for the addition of a new custom light deck.
# Data includes:
### .ipynb files containing the data analysis
### .json files containing the hard data produced during flight tests
### .py files containing the flight commands used to produce data

GCodeFiles #######################################
# Run XY_to_move_commands (jupyter notebook) to generate move commands from .csv file of (x, y) coordinates.

# Run image_to_gcode to generate a .csv file of (x, y) coordinates from a simple image.
    # 'pip install imageio' or 'conda install -c conda-forge imageio' if ae483 environment is active
    # Set working directory to same directory as the .png image and image_to_gcode.py file.
    # image_to_gcode requires constants.py to be in the same working directory.
    # Terminal Command to run imageio python script:
    python image_to_gcode.py --input xmas_tree.png --output xmas_tree.csv --threshold 100.