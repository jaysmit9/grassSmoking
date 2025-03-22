import psutil
import time
import sqlite3

def get_cpu_usage():
    cpu_percents = psutil.cpu_percent(interval=1, percpu=True)
    print(f"CPU usage per core: {cpu_percents}")
    return cpu_percents

def log_cpu_usage(cpu_percents):
    conn = sqlite3.connect('cpu_data.db')
    c = conn.cursor()
    c.execute('''CREATE TABLE IF NOT EXISTS cpu_data
                 (timestamp TEXT, core INTEGER, cpu_percent REAL)''')
    for core, cpu_percent in enumerate(cpu_percents):
        c.execute("INSERT INTO cpu_data (timestamp, core, cpu_percent) VALUES (datetime('now'), ?, ?)", (core, cpu_percent))
    conn.commit()
    conn.close()

def main():
    while True:
        cpu_percents = get_cpu_usage()
        log_cpu_usage(cpu_percents)
        time.sleep(1)  # Check every second

if __name__ == "__main__":
    main()