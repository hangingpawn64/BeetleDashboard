"""
Beetle Dashboard - Flask Application
Robot Telemetry Dashboard

Data Format (19 values as comma-separated string):
ax,ay,az,gx,gy,gz,enc1,enc2,enc3,enc4,enc5,enc6,tof1,tof2,tof3,tof4,tof5,tof6,tof7

Send data via POST to /update endpoint.
"""

from flask import Flask, render_template, jsonify, request
import threading
import time

app = Flask(__name__)

# ============================================
# SENSOR DATA STORAGE
# ============================================
sensor_data = {
    # MPU6050 Linear Acceleration (m/s²)
    'ax': 0.0,
    'ay': 0.0,
    'az': 0.0,
    # MPU6050 Angular Velocity (°/s)
    'gx': 0.0,
    'gy': 0.0,
    'gz': 0.0,
    # 6 Encoders (3 left wheels + 3 right wheels)
    'enc1': 0,
    'enc2': 0,
    'enc3': 0,
    'enc4': 0,
    'enc5': 0,
    'enc6': 0,
    # 7 ToF Sensors (distances in mm)
    'tof1': 0,
    'tof2': 0,
    'tof3': 0,
    'tof4': 0,
    'tof5': 0,
    'tof6': 0,
    'tof7': 0,
    # Calculated yaw from gz integration
    'yaw': 0.0,
}

# Thread lock for safe data access
data_lock = threading.Lock()

# Last update timestamp for yaw integration
last_update_time = time.time()


# ============================================
# DATA PARSING
# ============================================
def parse_sensor_string(data_string):
    """
    Parse comma-separated sensor data string.
    Format: ax,ay,az,gx,gy,gz,enc1,enc2,enc3,enc4,enc5,enc6,tof1,tof2,tof3,tof4,tof5,tof6,tof7
    """
    global sensor_data, last_update_time
    
    try:
        values = data_string.strip().split(',')
        
        if len(values) != 19:
            print(f"Error: Expected 19 values, got {len(values)}")
            return False
        
        # Calculate time delta for yaw integration
        current_time = time.time()
        dt = current_time - last_update_time
        last_update_time = current_time
        
        with data_lock:
            # MPU6050 - Linear Acceleration
            sensor_data['ax'] = float(values[0])
            sensor_data['ay'] = float(values[1])
            sensor_data['az'] = float(values[2])
            
            # MPU6050 - Angular Velocity
            sensor_data['gx'] = float(values[3])
            sensor_data['gy'] = float(values[4])
            sensor_data['gz'] = float(values[5])
            
            # 6 Encoders
            sensor_data['enc1'] = int(float(values[6]))
            sensor_data['enc2'] = int(float(values[7]))
            sensor_data['enc3'] = int(float(values[8]))
            sensor_data['enc4'] = int(float(values[9]))
            sensor_data['enc5'] = int(float(values[10]))
            sensor_data['enc6'] = int(float(values[11]))
            
            # 7 ToF Sensors
            sensor_data['tof1'] = int(float(values[12]))
            sensor_data['tof2'] = int(float(values[13]))
            sensor_data['tof3'] = int(float(values[14]))
            sensor_data['tof4'] = int(float(values[15]))
            sensor_data['tof5'] = int(float(values[16]))
            sensor_data['tof6'] = int(float(values[17]))
            sensor_data['tof7'] = int(float(values[18]))
            
            # Integrate gz to get yaw angle
            sensor_data['yaw'] += sensor_data['gz'] * dt
            sensor_data['yaw'] = sensor_data['yaw'] % 360
        
        return True
        
    except Exception as e:
        print(f"Parse error: {e}")
        return False


# ============================================
# FLASK ROUTES
# ============================================

@app.route('/')
def index():
    """Serve the main dashboard page."""
    return render_template('index.html')


@app.route('/data')
def get_data():
    """GET - Retrieve current sensor data as JSON."""
    with data_lock:
        return jsonify(sensor_data)


@app.route('/update', methods=['POST'])
def update_data():
    """
    POST - Receive sensor data from ESP/RPi.
    
    Format: ax,ay,az,gx,gy,gz,enc1,enc2,enc3,enc4,enc5,enc6,tof1,tof2,tof3,tof4,tof5,tof6,tof7
    """
    try:
        data_string = request.get_data(as_text=True)
        print(f"Received: {data_string}")
        
        if parse_sensor_string(data_string):
            return jsonify({'status': 'ok'}), 200
        else:
            return jsonify({'status': 'error', 'message': 'Parse failed'}), 400
            
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/reset', methods=['POST'])
def reset_position():
    """Reset yaw angle."""
    global sensor_data
    with data_lock:
        sensor_data['yaw'] = 0.0
    return jsonify({'status': 'ok'})


@app.route('/stream')
def video_stream():
    """Camera stream - not implemented."""
    return '', 404


# ============================================
# MAIN
# ============================================

if __name__ == '__main__':
    print("\n" + "="*60)
    print("  BEETLE DASHBOARD")
    print("="*60)
    print("  Dashboard:     http://localhost:5000")
    print("  Data API:      GET  http://localhost:5000/data")
    print("  Update Data:   POST http://localhost:5000/update")
    print("  Reset:         POST http://localhost:5000/reset")
    print("="*60)
    print("\n  Data Format (19 values, comma-separated):")
    print("  ax,ay,az,gx,gy,gz,enc1,enc2,enc3,enc4,enc5,enc6,")
    print("  tof1,tof2,tof3,tof4,tof5,tof6,tof7")
    print("\n  Test with curl:")
    print('  curl -X POST -d "1,2,3,4,5,6,100,101,102,103,104,105,50,60,70,80,90,100,110" http://localhost:5000/update')
    print("="*60 + "\n")
    
    app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)
