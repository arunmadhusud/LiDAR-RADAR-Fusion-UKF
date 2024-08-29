# plot NIS values using matplotlib library
import matplotlib.pyplot as plt
import numpy as np

def plot_nis(nis, threshold , title):
    plt.plot(nis)
    plt.axhline(y=threshold, color='r', linestyle='--')
    plt.xlabel('time step')
    plt.ylabel('NIS')
    plt.title('NIS values for ' + title)
    plt.show()