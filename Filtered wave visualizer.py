import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import csv

# Load the CSV files
data = pd.read_csv('filtered_sine_wave.csv')
data2 = pd.read_csv('noisy_sine_wave.csv')
data3 = pd.read_csv('clean_sine_wave.csv')

# Extract the columns to plot
x = data['time']
y = data[' filtered sine wave']
y2 = data2['Noisy Sine Wave']
y3 = data3['Clean Sine Wave']

plt.plot(x,y2)
plt.plot(x,y)
#plt.plot(x,y3)
plt.show()

