import serial
import numpy
import time
import re

from find_pos import global_position
from rover_commands import run_nearby_samples

# Strength we consider to be close enough


def main():
    # Define Data Arrays and Global Vars
    prev_samples = []
    prev_pos = []
    desired_strength = -34
    avg_sample = -999

    # Begin sampling nearby 
    run_nearby_samples()

    while (avg_sample < desired_strength):

        


        curr_pos = global_position()
        curr_strength = sample_strength()

        prev_pos.append(curr_pos)
        prev_samples.append(curr_strength)




    return

main()