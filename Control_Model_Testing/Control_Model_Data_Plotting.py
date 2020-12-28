# -*- coding: utf-8 -*-
# Import required modules
# --------------------------------------------------------------------------- #
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 15})  # Set matplotlib font size
import subprocess
# --------------------------------------------------------------------------- #


# Functions
# --------------------------------------------------------------------------- #
def run_simulation(simulation_executable):
    '''Runs the simulation executable.'''
    subprocess.run(["./" + simulation_executable])


def file_read(filename):
    '''Takes as input a filename and reads the data from the file line by
    line. Data from the file is read into arrays for storing the simulation
    results for the Scout, a target aircraft and the modified Scout.'''

    # Initialse arrays for storing data
    time = []
    horizontal_velocity_sc = []
    vertical_velocity_sc = []
    pitch_rate_sc = []
    pitch_angle_sc = []
    horizontal_velocity_t = []
    vertical_velocity_t = []
    pitch_rate_t = []
    pitch_angle_t = []
    horizontal_velocity_sc_mod = []
    vertical_velocity_sc_mod = []
    pitch_rate_sc_mod = []
    pitch_angle_sc_mod = []

    # Open the file and read in the data
    with open(filename, 'r') as f:
        # read in data line by line into the 1D array lines
        lines = f.readlines()

        # Go through each line read in from the file
        for line in lines:
            # Read in the discrete time values
            time.append(float(line.split(' ')[0]))

            # Read in the values for the Scout
            horizontal_velocity_sc.append(float(line.split(' ')[1]))
            vertical_velocity_sc.append(float(line.split(' ')[2]))
            pitch_rate_sc.append(float(line.split(' ')[3]))
            pitch_angle_sc.append(float(line.split(' ')[4]))

            # Read in the values for the target aircraft
            horizontal_velocity_t.append(float(line.split(' ')[5]))
            vertical_velocity_t.append(float(line.split(' ')[6]))
            pitch_rate_t.append(float(line.split(' ')[7]))
            pitch_angle_t.append(float(line.split(' ')[8]))

            # Read in the values for the modified Scout
            horizontal_velocity_sc_mod.append(float(line.split(' ')[9]))
            vertical_velocity_sc_mod.append(float(line.split(' ')[10]))
            pitch_rate_sc_mod.append(float(line.split(' ')[11]))
            pitch_angle_sc_mod.append(float(line.split(' ')[12]))
    return(time, horizontal_velocity_sc, vertical_velocity_sc, pitch_rate_sc,
           pitch_angle_sc, horizontal_velocity_t, vertical_velocity_t,
           pitch_rate_t, pitch_angle_t, horizontal_velocity_sc_mod,
           vertical_velocity_sc_mod, pitch_rate_sc_mod, pitch_angle_sc_mod)
# --------------------------------------------------------------------------- #

# Run the external simulation code
# --------------------------------------------------------------------------- #
run_simulation(".//build//source//Control_model")
# --------------------------------------------------------------------------- #

# Read in the data from the simulation run
# --------------------------------------------------------------------------- #
times, horizontal_velocites_sc, vertical_velocities_sc, pitch_rates_sc, \
       pitch_angles_sc, horizontal_velocites_t, vertical_velocities_t, \
       pitch_rates_t, pitch_angles_t, horizontal_velocites_sc_mod, \
       vertical_velocities_sc_mod, pitch_rates_sc_mod, pitch_angles_sc_mod\
       = file_read("test_data.txt")

# Plot the data from the simulation
# --------------------------------------------------------------------------- #
# Create subplot of 4 plots, organised in 2 rows and 2 columns
fig, axs = plt.subplots(2, 2)

# Plot horizontal velocities
axs[0][0].plot(times, horizontal_velocites_sc, 'k', label="Scout")
axs[0][0].plot(times, horizontal_velocites_t, label="Target Aircraft")
axs[0][0].plot(times, horizontal_velocites_sc_mod, 'r--',
               label="Modified Scout")
axs[0][0].set_title("Horizontal Velocity Vs Time")
axs[0][0].set_ylabel(r"$u$ ($m/s$)")
axs[0][0].legend()

# Plot vertical velocities
axs[0][1].plot(times, vertical_velocities_sc, 'k', label="Scout")
axs[0][1].plot(times, vertical_velocities_t, label="Target Aircraft")
axs[0][1].plot(times, vertical_velocities_sc_mod, 'r--',
               label="Modified Scout")
axs[0][1].set_title("Vertical Velocity Vs Time")
axs[0][1].set_ylabel(r"$w$ ($m/s$)")
axs[0][1].legend()

# Plot pitch rates
axs[1][0].plot(times, pitch_rates_sc, 'k', label="Scout")
axs[1][0].plot(times, pitch_rates_t, label="Target Aircraft")
axs[1][0].plot(times, pitch_rates_sc_mod, 'r--', label="Modified Scout")
axs[1][0].set_title("Pitch Rate Vs Time")
axs[1][0].set_ylabel(r"$q$ ($rad/s$)")
axs[1][0].legend()

# Plot pitch angles
axs[1][1].plot(times, pitch_angles_sc, 'k', label="Scout")
axs[1][1].plot(times, pitch_angles_t, label="Target Aircraft")
axs[1][1].plot(times, pitch_angles_sc_mod, 'r--', label="Modified Scout")
axs[1][1].set_title("Pitch Angle Vs Time")
axs[1][1].set_ylabel(r"$\theta$ ($rad$)")
axs[1][1].legend()

# Place labels on the x-axis of the subplots
for ax in axs.flat:
    ax.set(xlabel=r'time ($s$)')

# Adjust the horizontal spacing between subplots
plt.subplots_adjust(hspace=0.5)
# --------------------------------------------------------------------------- #

# Calculate the average absolute error between the target aircraft and the
# modified Scout
# --------------------------------------------------------------------------- #
print(abs(sum([pitch_rates_t[i]-pitch_rates_sc_mod[i] for i in
           range(len(pitch_rates_sc_mod))]))/len(pitch_rates_sc_mod))
# --------------------------------------------------------------------------- #
