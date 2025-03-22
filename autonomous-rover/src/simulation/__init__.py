# Import and expose the simulation functions
from .simulator import simulate, plot_results, create_animation

# Optional - expose the simulation hardware components
from .sim_gps import SimulatedGPS
from .sim_motors import SimulatedMotorController