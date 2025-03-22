from flask import Flask, render_template, request, jsonify
import requests
import json
from basic_path_planner import BasicPathPlanner

app = Flask(__name__)

@app.route('/')
def index():
    try:
        # Fetch data from the REST API
        response = requests.get('http://10.0.0.90:5000/get_dashboard')
        response.raise_for_status()  # Raise an exception for HTTP errors
        data_json = response.json()
    except requests.exceptions.RequestException as e:
        print(f"Error fetching data: {e}")
        data_json = {
            "current_coordinates": {"lat": 0, "lon": 0},
            "historical_coords": [],
            "cpu_usage": [],
            "historical_cpu": []
        }
    except requests.exceptions.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
        print(f"Response content: {response.text}")
        data_json = {
            "current_coordinates": {"lat": 0, "lon": 0},
            "historical_coords": [],
            "cpu_usage": [],
            "historical_cpu": []
        }

    # Extract current coordinates and historical data
    current_coordinates = data_json['current_coordinates']
    historical_coords = data_json['historical_coords']
    historical_cpu = data_json['historical_cpu']

    # Convert historical data to the format expected by the template
    gps_data = [(entry['timestamp'], entry['lat'], entry['lon']) for entry in historical_coords]
    cpu_data = [(entry['timestamp'], entry['core'], entry['cpu_percent']) for entry in historical_cpu]

    return render_template('dashboard.html', gps_data=gps_data, cpu_data=cpu_data, current_coordinates=current_coordinates)

@app.route('/data')
def data():
    try:
        # Fetch data from the REST API
        response = requests.get('http://10.0.0.90:5000/get_dashboard')
        response.raise_for_status()  # Raise an exception for HTTP errors
        data_json = response.json()
    except requests.exceptions.RequestException as e:
        print(f"Error fetching data: {e}")
        data_json = {
            "current_coordinates": {"lat": 0, "lon": 0},
            "historical_coords": [],
            "cpu_usage": [],
            "historical_cpu": []
        }
    except requests.exceptions.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
        print(f"Response content: {response.text}")
        data_json = {
            "current_coordinates": {"lat": 0, "lon": 0},
            "historical_coords": [],
            "cpu_usage": [],
            "historical_cpu": []
        }

    # Extract current coordinates and historical data
    current_coordinates = data_json['current_coordinates']
    historical_coords = data_json['historical_coords']
    historical_cpu = data_json['historical_cpu']

    # Convert historical data to the format expected by the template
    gps_data = [(entry['timestamp'], entry['lat'], entry['lon']) for entry in historical_coords]
    cpu_data = [(entry['timestamp'], entry['core'], entry['cpu_percent']) for entry in historical_cpu]

    return jsonify(gps_data=gps_data, cpu_data=cpu_data, current_coordinates=current_coordinates)

@app.route('/polygon_data')
def polygon_data():
    try:
        with open('polygon_data.json', 'r') as f:
            polygon_data = json.load(f)
    except FileNotFoundError:
        polygon_data = []
    return jsonify(polygon_data)

@app.route('/save_polygon', methods=['POST'])
def save_polygon():
    polygon_data = request.json.get('polygon', [])
    # Save the polygon data to a file or database
    with open('polygon_data.json', 'w') as f:
        json.dump(polygon_data, f)
    return jsonify({"status": "success", "message": "Polygon saved successfully"})

@app.route('/plan_basic_path', methods=['POST'])
def plan_basic_path():
    try:
        with open('polygon_data.json', 'r') as f:
            polygon_data = json.load(f)
        print('Polygon data:', polygon_data)  # Debugging statement
        polygon = [(p['lat'], p['lon']) for p in polygon_data]
        print('Polygon coordinates:', polygon)  # Debugging statement

        # Get the user-defined starting point and angle point from the request
        start_point = request.json.get('start_point', None)
        if start_point:
            start_point = (start_point['lat'], start_point['lon'])
        print('Start point:', start_point)  # Debugging statement

        angle_point = request.json.get('angle_point', None)
        if angle_point:
            angle_point = (angle_point['lat'], angle_point['lon'])
        print('Angle point:', angle_point)  # Debugging statement

        planner = BasicPathPlanner(polygon)
        planner.plan_basic_path(start_point=start_point, angle_point=angle_point)
        path = planner.get_path()
        print('Generated path:', path)  # Debugging statement
        path_data = [{'lat': p[0], 'lon': p[1]} for p in path]
        print('Path data to return:', path_data)  # Debugging statement
        # Save the path data to a file
        with open('path_data.json', 'w') as f:
            json.dump(path_data, f)
    except FileNotFoundError:
        path_data = []
    except Exception as e:
        print(f"Error in plan_basic_path: {e}")
        return jsonify({"error": str(e)}), 500
    return jsonify(path_data)

if __name__ == '__main__':
    app.run(debug=True, port=5001)  # Change the port number here