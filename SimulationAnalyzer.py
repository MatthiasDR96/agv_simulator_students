import configparser

import matplotlib.pyplot as plt
import numpy as np


class SimulationAnalyzer:

    def __init__(self):

        # Setup file
        setup = configparser.ConfigParser()
        setup.read('test_vectors/setup.ini')

        # Logfiles
        self.tasks_executing_logfile = open("logfiles/tasks_executing.txt", "r")
        self.global_robot_list_logfile = open("logfiles/global_robot_list.txt", "r")
        self.local_task_list_logfile = open("logfiles/local_task_lists.txt", "r")

        # Simulation params
        self.number_of_agvs = int(setup['GENERAL']['number_of_agvs'])
        duration = open("logfiles/simulation_duration.txt", "r")
        self.simulation_duration = int(duration.readline())
        duration.close()

        # Storage arrays
        self.agv_charge = [np.zeros((self.simulation_duration, 1)) for i in range(self.number_of_agvs)]
        self.agv_load = [np.zeros((self.simulation_duration, 1)) for i in range(self.number_of_agvs)]
        self.agv_idle = [np.zeros((self.simulation_duration, 1)) for i in range(self.number_of_agvs)]

        # Analyzers
        self.agv_status_analyzer()

        # Close log files
        self.local_task_list_logfile.close()
        self.tasks_executing_logfile.close()
        self.global_robot_list_logfile.close()

    def task_to_execute_analyzer(self):

        for i in range(self.simulation_duration):

            line = self.local_task_list_logfile.readline()
            local_task_lists = list(line.split('/')[1].split(':'))
            for list_ in local_task_lists:
                if not list_ == "[]":
                    list_ = list_[2:-2].split(', ')
                    for i in range(len(list_)):
                        task = list_[i].split(';')
                        print(task)

    def agv_status_analyzer(self):

        # Get data from log files
        for i in range(self.simulation_duration):
            line = self.global_robot_list_logfile.readline()
            global_robot_list = list(line.split('/')[1].split(':'))
            for j in range(len(global_robot_list) - 1):
                robot = global_robot_list[j].split(';')
                if robot[3] == 'CHARGING':
                    self.agv_charge[j][i] = 1
                if robot[3] == 'IDLE':
                    self.agv_idle[j][i] = 1
                if robot[3] == 'BUSY' or robot[3] == 'EMPTY':
                    self.agv_load[j][i] = 1

        # Plot data
        plt.figure(1)
        plt.legend(['BUSY', 'CHARGING', 'IDLE'])
        time = np.linspace(1, self.simulation_duration, self.simulation_duration)
        for k in range(self.number_of_agvs):
            plt.subplot(self.number_of_agvs, 1, k + 1)
            plt.ylabel("AGV " + str(k + 1))
            plt.axis([0, self.simulation_duration, 0, 1.1])
            plt.plot(time, self.agv_load[k])
            plt.plot(time, self.agv_charge[k])
            plt.plot(time, self.agv_idle[k])
        plt.xlabel("Simulation time")
        plt.subplots_adjust(wspace=1.5, hspace=0.4)
        plt.show()

        # Print results
        for k in range(self.number_of_agvs):
            busy = sum(self.agv_load[k])
            busy_percentage = round((busy / self.simulation_duration) * 100, 2)
            charging = sum(self.agv_charge[k])
            charging_percentage = round((charging / self.simulation_duration) * 100, 2)
            idle = sum(self.agv_idle[k])
            idle_percentage = round((idle / self.simulation_duration) * 100, 2)
            print('\nAGV ' + str(k + 1) + ':')
            print('\tPercentage busy: ' + str(busy_percentage) + '%')
            print('\tPercentage charging: ' + str(charging_percentage) + '%')
            print('\tPercentage idle: ' + str(idle_percentage) + '%')
            print('\tTotal: ' + str(idle_percentage + charging_percentage + busy_percentage) + '%')


analyzer = SimulationAnalyzer()
