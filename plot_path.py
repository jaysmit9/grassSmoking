import requests
import matplotlib.pyplot as plt

def fetch_path_data():
    response = requests.get('http://localhost:5001/plan_path')
    response.raise_for_status()  # Raise an exception for HTTP errors
    path_data = response.json()
    print(f"Fetched path data: {path_data}")  # Debugging statement
    return path_data

def plot_path(path_data):
    if not path_data:
        print("No path data to plot.")  # Debugging statement
        return

    latitudes = [point['lat'] for point in path_data]
    longitudes = [point['lon'] for point in path_data]

    print(f"Latitudes: {latitudes}")  # Debugging statement
    print(f"Longitudes: {longitudes}")  # Debugging statement

    plt.figure(figsize=(10, 6))
    plt.plot(longitudes, latitudes, marker='o', linestyle='-', color='b')
    plt.title('Planned Path')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    path_data = fetch_path_data()
    plot_path(path_data)