# -*- coding: utf-8 -*-
# Import required modules
# --------------------------------------------------------------------------- #
import subprocess
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 15})  # Set matplotlib font size
# --------------------------------------------------------------------------- #


data_dir = ".//build//source//"


# --------------------------------------------------------------------------- #
class sim_analysis():
    '''
    Provides functionality for analysing simulation results. Takes as input
    strings for the simulation executable to run and the filename of the data
    output from the simulation.
    '''

    def __init__(self, simulation_executable, data_filename):
        self.simulation_executable = simulation_executable
        self.data_filename = data_filename
        self.control_inputs_data_filename = "test_data_aircraft_inputs.txt"

        # Initialise arrays for storing data
        self.times = []
        self.horizontal_velocities_sc = []
        self.vertical_velocities_sc = []
        self.pitch_rates_sc = []
        self.pitch_angles_sc = []
        self.horizontal_velocities_t = []
        self.vertical_velocities_t = []
        self.pitch_rates_t = []
        self.pitch_angles_t = []
        self.horizontal_velocities_sc_mod = []
        self.vertical_velocities_sc_mod = []
        self.pitch_rates_sc_mod = []
        self.pitch_angles_sc_mod = []

        self.control_sc_and_t = []
        self.control_sc_mod = []

    def run_simulation(self):
        '''
        Runs the simulation executable.
        '''

        subprocess.run(["./" + self.simulation_executable])

    def file_read_state_data(self):
        '''
        Takes as input a filename and reads the data from the file line by
        line. Data from the file is read into arrays for storing the simulation
        results for the Scout, a target aircraft and the modified Scout.
        '''

        # Open the file and read in the data
        with open(self.data_filename, 'r') as f:
            # read in data line by line into the 1D array lines
            lines = f.readlines()

            # Go through each line read in from the file
            for line in lines:
                # Read in the discrete time values
                self.times.append(float(line.split(' ')[0]))

                if self.data_filename == "test_data_full_model.txt":
                    # Read in the values for the Scout
                    self.horizontal_velocities_sc.append(
                            float(line.split(' ')[1]))
                    self.vertical_velocities_sc.append(
                            float(line.split(' ')[2]))
                    self.pitch_rates_sc.append(float(line.split(' ')[3]))
                    self.pitch_angles_sc.append(float(line.split(' ')[4]))

                    # Read in the values for the target aircraft
                    self.horizontal_velocities_t.append(
                            float(line.split(' ')[5]))
                    self.vertical_velocities_t.append(
                            float(line.split(' ')[6]))
                    self.pitch_rates_t.append(float(line.split(' ')[7]))
                    self.pitch_angles_t.append(float(line.split(' ')[8]))

                    # Read in the values for the modified Scout
                    self.horizontal_velocities_sc_mod.append(
                            float(line.split(' ')[9]))
                    self.vertical_velocities_sc_mod.append(
                            float(line.split(' ')[10]))
                    self.pitch_rates_sc_mod.append(float(line.split(' ')[11]))
                    self.pitch_angles_sc_mod.append(float(line.split(' ')[12]))
                if self.data_filename == "test_data_spo_model.txt":
                    # Read in the values for the Scout
                    self.vertical_velocities_sc.append(
                            float(line.split(' ')[1]))
                    self.pitch_rates_sc.append(float(line.split(' ')[2]))

                    # Read in the values for the target aircraft
                    self.vertical_velocities_t.append(
                            float(line.split(' ')[3]))
                    self.pitch_rates_t.append(float(line.split(' ')[4]))

                    # Read in the values for the modified Scout
                    self.vertical_velocities_sc_mod.append(
                            float(line.split(' ')[5]))
                    self.pitch_rates_sc_mod.append(float(line.split(' ')[6]))

    def file_read_control_inputs_data(self):
        '''
        Takes as input a filename and reads the data from the file line by
        line. Data from the file is read into arrays for storing the control
        inputs for the Scout and target aircraft, and the modified Scout.
        '''

        with open(self.control_inputs_data_filename, 'r') as f:
            # read in data line by line into the 1D array lines
            lines = f.readlines()

            # Go through each line read in from the file
            for line in lines:
                # Read in the values for control inputs
                self.control_sc_and_t.append(float(line.split(' ')[1]))
                self.control_sc_mod.append(float(line.split(' ')[2]))

    def data_plotting(self):
        '''
        Plots the data read in from the simulation results. For the SPO results
        only the vertical velocities and pitch rates are plotted. For the full
        longitudinal model the vertical velocities, pitch rates, horizontal
        velocities and pitch angles are plotted. Results are plotted for the
        Scout, target aircraft and the modified Scout.
        '''

        # Create subplot of 4 plots, organised in 2 rows and 2 columns
        fig, axs = plt.subplots(2, 2)

        if self.data_filename == "test_data_spo_model.txt":
            # Plot vertical velocities
            axs[0][1].plot(self.times, self.vertical_velocities_sc, 'k',
                           label="Scout")
            axs[0][1].plot(self.times, self.vertical_velocities_t,
                           label="Target Aircraft")
            axs[0][1].plot(self.times, self.vertical_velocities_sc_mod, 'r--',
                           label="Modified Scout")
            axs[0][1].set_title("Vertical Velocity Vs Time")
            axs[0][1].set_ylabel(r"$w$ ($m/s$)")
            axs[0][1].legend()

            # Plot pitch rates
            axs[1][0].plot(self.times, self.pitch_rates_sc, 'k', label="Scout")
            axs[1][0].plot(self.times, self.pitch_rates_t,
                           label="Target Aircraft")
            axs[1][0].plot(self.times, self.pitch_rates_sc_mod, 'r--',
                           label="Modified Scout")
            axs[1][0].set_title("Pitch Rate Vs Time")
            axs[1][0].set_ylabel(r"$q$ ($rad/s$)")
            axs[1][0].legend()

        if self.data_filename == "test_data_full_model.txt":
            # Plot vertical velocities
            axs[0][1].plot(self.times, self.vertical_velocities_sc, 'k',
                           label="Scout")
            axs[0][1].plot(self.times, self.vertical_velocities_t,
                           label="Target Aircraft")
            axs[0][1].plot(self.times, self.vertical_velocities_sc_mod, 'r--',
                           label="Modified Scout")
            axs[0][1].set_title("Vertical Velocity Vs Time")
            axs[0][1].set_ylabel(r"$w$ ($m/s$)")
            axs[0][1].legend()

            # Plot pitch rates
            axs[1][0].plot(self.times, self.pitch_rates_sc, 'k', label="Scout")
            axs[1][0].plot(self.times, self.pitch_rates_t,
                           label="Target Aircraft")
            axs[1][0].plot(self.times, self.pitch_rates_sc_mod, 'r--',
                           label="Modified Scout")
            axs[1][0].set_title("Pitch Rate Vs Time")
            axs[1][0].set_ylabel(r"$q$ ($rad/s$)")
            axs[1][0].legend()

            # Plot horizontal velocities
            axs[0][0].plot(self.times, self.horizontal_velocities_sc, 'k',
                           label="Scout")
            axs[0][0].plot(self.times, self.horizontal_velocities_t,
                           label="Target Aircraft")
            axs[0][0].plot(self.times, self.horizontal_velocities_sc_mod,
                           'r--', label="Modified Scout")
            axs[0][0].set_title("Horizontal Velocity Vs Time")
            axs[0][0].set_ylabel(r"$u$ ($m/s$)")
            axs[0][0].legend()

            # Plot pitch angles
            axs[1][1].plot(self.times, self.pitch_angles_sc, 'k',
                           label="Scout")
            axs[1][1].plot(self.times, self.pitch_angles_t,
                           label="Target Aircraft")
            axs[1][1].plot(self.times, self.pitch_angles_sc_mod, 'r--',
                           label="Modified Scout")
            axs[1][1].set_title("Pitch Angle Vs Time")
            axs[1][1].set_ylabel(r"$\theta$ ($rad$)")
            axs[1][1].legend()

        # Place labels on the x-axis of the subplots
        for ax in axs.flat:
            ax.set(xlabel=r'time ($s$)')

        # Adjust the horizontal spacing between subplots
        plt.subplots_adjust(hspace=0.5)

    def elevator_deflection_plotting(self):
        '''
        Plots the data read in from the simulation results. Plots the control
        input for the Scout and target aircraft, and the control input for the
        modified Scout.
        '''

        # Create subplot of 1 plot
        fig, axs = plt.subplots(1)

        # Plot control inputs
        axs.plot(self.times, self.control_sc_and_t, 'k',
                 label="Scout and Target Aircraft")
        axs.plot(self.times, self.control_sc_mod, 'r--',
                 label="Modified Scout")
        axs.set_title("Control Inputs Vs Time")
        axs.set_xlabel(r'time ($s$)')
        axs.set_ylabel(r"$u$ ($rad$)")
        axs.legend()

    def calc_error(self):
        '''
        Calculates the average absolute error between the target aircraft and
        the modified Scout.
        '''

        print(abs(sum([self.pitch_rates_t[i]-self.pitch_rates_sc_mod[i]
                       for i in range(len(self.pitch_rates_sc_mod))])) /
              len(self.pitch_rates_sc_mod))
# --------------------------------------------------------------------------- #


# Functions
# --------------------------------------------------------------------------- #
def full_model_sim():
    '''
    Analyses the results for the full longitudinal model.
    '''

    full_model_results = sim_analysis(data_dir +
                                      "sim_runner_full_longitudinal_model",
                                      "test_data_full_model.txt")
    full_model_results.run_simulation()  # Run the external simulation code
    full_model_results.file_read_state_data()  # Read in the data
    full_model_results.file_read_control_inputs_data()  # Read in the data
    full_model_results.data_plotting()  # Plot the data from the simulation
    full_model_results.elevator_deflection_plotting()  # Plot the control data
    full_model_results.calc_error()  # Error between modified Scout and target


def spo_model_sim():
    '''
    Analyses the results for the SPO model.
    '''

    spo_model_results = sim_analysis(data_dir + "sim_runner_spo_model",
                                     "test_data_spo_model.txt")
    spo_model_results.run_simulation()  # Run the external simulation code
    spo_model_results.file_read_state_data()  # Read in the data
    spo_model_results.file_read_control_inputs_data()  # Read in the data
    spo_model_results.data_plotting()  # Plot the data from the simulation
    spo_model_results.elevator_deflection_plotting()  # Plot the control data
    spo_model_results.calc_error()  # Error between modified Scout and target
# --------------------------------------------------------------------------- #


full_model_sim()
spo_model_sim()
