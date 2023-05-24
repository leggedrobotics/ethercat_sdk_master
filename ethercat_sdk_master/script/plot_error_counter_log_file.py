import argparse
import pandas as pd
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description='Plot error counter log file')
parser.add_argument('log_files', metavar='log_files', type=str, nargs='+', help='log file(s) to plot, plots all error counters for all slaves')
parser.add_argument('--slave', metavar='slave', type=str, nargs='+', help='slave to plot, plots all error counters for this slave')
parser.add_argument('--save_to_file', metavar='save_to_file', type=bool, nargs='+', help='should plots be saved to file')
parser.add_argument('--outputpath', metavar='output_path', type=str, default="./log_analysis", help='output path for the plots to be saved, default current directory')
args = parser.parse_args()

filenames = args.log_files
specificSlave = args.slave
outputpath = args.outputpath

if outputpath:
  import os
  if not os.path.exists(outputpath):
    os.makedirs(outputpath)
  if not outputpath.endswith("/"):
    outputpath += "/"

for filename in filenames:
  out = pd.read_csv(filename, header=[0,1], delimiter=", ", engine='python')
  header = out.columns
  slaveList = header.get_level_values(0)[2:].unique()
  time = out[header.get_level_values(0)[0]].values

  busName = header.get_level_values(0)[1]
  print("\nReading Logfile: {},\nFound the following slaves on bus {}:".format( filename, busName))
  for slave in slaveList:
    print(slave)
  if not specificSlave:
    print("\nPlotting all error counters for all slaves and saves them to {}".format(outputpath))
    for slave in slaveList:
        slaveData = out[slave]
        for i, errorCounter in enumerate(slaveData.columns):
            fig = plt.figure()
            axes = slaveData[errorCounter].plot()
            axes.set_title("Slave: " + slave+"\n"+errorCounter)
            axes.set_xlabel("Time [s]")
            axes.set_ylabel(errorCounter)
            axes.grid()
            plt.savefig(outputpath + filename+"_"+slave+"_"+errorCounter+".png")
            plt.close(fig)
  else:
    print("\nPlotting all error counters for slave(s): {} and saves them to {}".format(specificSlave, outputpath))
    for slave in specificSlave:
        if slave not in slaveList:
            print("Slave {} not found in logfile {}".format(slave, filename))
            continue
        slaveData = out[slave]
        for i, errorCounter in enumerate(slaveData.columns):
            fig, ax  = plt.subplots()
            ax.plot(time, slaveData[errorCounter].values)
            ax.set_title("Slave: " + slave+"\n"+errorCounter)
            ax.set_xlabel("Time [s]")
            ax.set_ylabel(errorCounter)
            ax.grid()
            plt.savefig(outputpath + filename+"_"+slave+"_"+errorCounter+".png")
            plt.close(fig)

  