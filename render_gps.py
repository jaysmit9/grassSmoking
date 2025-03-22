from flask import Flask, render_template, jsonify
import folium
import json
import random
import threading
import time
import requests

app = Flask(__name__)

# Global variable to store the latest coordinates
latest_coord = [37.7749, -122.4194]  # Initial coordinates (San Francisco)

# Function to simulate getting the latest GPS coordinates
def get_latest_coordinates():
    global latest_coord
    while True:
        # Replace this with actual code to get the latest GPS coordinates

        # URL of the REST API endpoint
        url = 'http://10.0.0.90:5000'
        
        status_code = 500
        while status_code == 500:
            # Make a GET request to the API
            response = requests.get(url)
            status_code = response.status_code
            time.sleep(1)

        # Check if the request was successful
        if response.status_code == 200:
            # Parse the JSON response
            data = str(response.json())
            data = data.replace("\'", "\"")
            coordinates_dict = json.loads(data)
        else:
            print("Failed to retrieve data. Status code:", response.status_code)

        #lat, lon = 37.7749, -122.4194  # Central point (San Francisco)
        lat = coordinates_dict["lat"]
        lon = coordinates_dict["lon"]
        latest_coord = [lat, lon]
        time.sleep(5)  # Update every 5 seconds

# Route to serve the map
@app.route('/')
def index():
    return render_template('index.html')

# Route to get the latest coordinates
@app.route('/latest-coordinates')
def latest_coordinates():
    return jsonify(latest_coord)

# Start the thread to update coordinates
threading.Thread(target=get_latest_coordinates, daemon=True).start()

if __name__ == '__main__':
    app.run(debug=True)
