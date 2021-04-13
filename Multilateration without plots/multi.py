#from multilaterate import get_loci as getLoci
import numpy as np
from scipy.optimize import least_squares as leastSquares

num_nodes = 4 # the number of nodes
target_range = 150 # length of a square contining the tracking target. Centered around (0, 0)
v = (3e8)/1.0003 # Speed of light in air is about 1.0003 times slower than in a vacuum
t_0 = 0


rt_noise = 1e-9 # standard deviation of assumed noise

nodes = np.array([[75, 75], [75, -75], [-75, 75], [-75, -75]]) # Node coordinates
print(f'towers:\n{nodes}')

tx = (np.random.rand(2) - 0.5) * target_range # randomizing the target's location
print(f'tx: {tx}')

dist = np.array([((x[0]-tx[0])**2 + (x[1]-tx[1])**2)**0.5 for x in nodes]) # Gets the distances between nodes
print(f'distances: {dist}')

rec_times = dist/v + t_0 # getting the time at which each node receives the transmission
# in practice, this would likely be found using the timestamps
rec_times += np.random.normal(loc=0, scale=rt_noise, size=num_nodes) # Does some funky stuff with noise
print(f'rec times: {rec_times}')

# Actual math that solves for the target's location
c = np.argmin(rec_times)
p_c = np.expand_dims(nodes[c], axis=0)
t_c = rec_times[c]

all_p_i = np.delete(nodes, c, axis=0) # Removes node c for vectorization
all_t_i = np.delete(rec_times, c, axis=0) # We may want to do this differently in practice


def eval_solution(x):
    """ x is 2 element array of x, y of the transmitter"""
    return (
          np.linalg.norm(x - p_c, axis=1)
        - np.linalg.norm(x - all_p_i, axis=1)
        + v*(all_t_i - t_c)
    )

# Initial guess.
# We could use the previous position here
x_init = [0, 0]

# Finds an x value that minimizes eval_solution
res = leastSquares(eval_solution, x_init)

print(f"Actual emitter location:    ({tx[0]:.1f}, {tx[1]:.1f}) ")
print(f"Calculated emitter locaion: ({res.x[0]:.1f}, {res.x[1]:.1f})")
print(f"Error in metres:            {np.linalg.norm(tx-res.x):.1f}")

wait = input('\n') # keeps the console from closing (unless I screw up)
