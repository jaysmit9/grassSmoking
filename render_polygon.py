from flask import Flask, render_template, jsonify, request
import json

app = Flask(__name__)

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

@app.route('/polygon_map')
def polygon_map():
    return render_template('polygon_map.html')

if __name__ == '__main__':
    app.run(debug=True, port=5002)  # Change the port number here