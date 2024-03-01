import numpy as np
import matplotlib.pyplot as plt

def calculate_eta_d(dimension, epsilon):
    return (1 - ((1 - epsilon) ** dimension) )

def compute_plot_eta_d():
    dimensions = range(2, 11)
    epsilons = [0.2, 0.1, 0.01]

    for epsilon in epsilons:
        eta_values = [calculate_eta_d(d, epsilon) for d in dimensions]
        plt.plot(dimensions, eta_values, label=f'ε={epsilon}')

    plt.xlabel('Dimension (d)')
    plt.ylabel('Fraction of Volume (ηd)')
    plt.title('Fraction of Volume vs Dimension')
    plt.legend()
    plt.grid(True)
    plt.show()
    
compute_plot_eta_d()