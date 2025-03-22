import matplotlib.pyplot as plt
import pandas as pd

def plot_trajectory(csv_file):
    """Plots the trajectory from a CSV file"""
    try:
        # Read the CSV file into a Pandas DataFrame
        df = pd.read_csv(csv_file)
        
        # Extract latitude and longitude values
        latitudes = df['lat']
        longitudes = df['lon']
        
        # Create the plot
        plt.figure(figsize=(10, 6))
        plt.plot(longitudes, latitudes, 'b-', label='Trajectory')
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.title('Rover Trajectory')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')  # Ensure the aspect ratio is equal
        plt.show()
    
    except FileNotFoundError:
        print(f"Error: File not found: {csv_file}")
    except KeyError as e:
        print(f"Error: Missing column in CSV file: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # Replace with the path to your CSV file
    csv_file = '/home/jay/projects/grassSmoking/autonomous-rover/src/trajectory_20250310_175816.csv'
    plot_trajectory(csv_file)