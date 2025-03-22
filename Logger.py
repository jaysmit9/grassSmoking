import logging
import sqlite3
from logging.handlers import RotatingFileHandler

# Set up logging
logger = logging.getLogger('GPSLogger')
logger.setLevel(logging.INFO)
handler = RotatingFileHandler('gps_data.log', maxBytes=2000, backupCount=5)
logger.addHandler(handler)

# Set up SQLite database
conn = sqlite3.connect('gps_data.db')
c = conn.cursor()
c.execute('''CREATE TABLE IF NOT EXISTS gps_data
             (timestamp TEXT, latitude REAL, longitude REAL)''')
conn.commit()

def log_gps_data(latitude, longitude):
    # Log to file
    logger.info(f'Latitude: {latitude}, Longitude: {longitude}')

    # Log to SQLite database
    c.execute("INSERT INTO gps_data (timestamp, latitude, longitude) VALUES (datetime('now'), ?, ?)",
              (latitude, longitude))
    conn.commit()

# Example usage
log_gps_data(37.7738, -122.4194)