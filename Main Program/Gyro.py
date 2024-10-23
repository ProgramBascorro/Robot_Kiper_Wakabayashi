import smbus2 as smbus
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# MPU-6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Initialize I2C bus
bus = smbus.SMBus(1)  # or smbus.SMBus(0) for older Raspberry Pi versions

def initialize_mpu():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value

def calibrate_gyro():
    gx_offset = 0
    gy_offset = 0
    gz_offset = 0
    num_samples = 1000
    
    print("Calibrating gyroscope... Please keep the sensor stationary.")
    for _ in range(num_samples):
        gx_offset += read_raw_data(GYRO_XOUT_H)
        gy_offset += read_raw_data(GYRO_YOUT_H)
        gz_offset += read_raw_data(GYRO_ZOUT_H)
        time.sleep(0.001)
    
    gx_offset /= num_samples
    gy_offset /= num_samples
    gz_offset /= num_samples
    
    return gx_offset, gy_offset, gz_offset

def get_gyro_data(gx_offset, gy_offset, gz_offset):
    gx = (read_raw_data(GYRO_XOUT_H) - gx_offset) / 131.0
    gy = (read_raw_data(GYRO_YOUT_H) - gy_offset) / 131.0
    gz = (read_raw_data(GYRO_ZOUT_H) - gz_offset) / 131.0
    return gx, gy, gz

# Initialize MPU and calibrate
initialize_mpu()
gx_offset, gy_offset, gz_offset = calibrate_gyro()

# Variables to keep track of angles
angle_x = 0
angle_y = 0
angle_z = 0
previous_time = time.time()

def update(frame):
    global angle_x, angle_y, angle_z, previous_time

    try:
        current_time = time.time()
        dt = current_time - previous_time
        previous_time = current_time

        gx, gy, gz = get_gyro_data(gx_offset, gy_offset, gz_offset)

        angle_x += gx * dt
        angle_y += gy * dt
        angle_z += gz * dt

        print(f"Gyroscope angles - X: {angle_x:.2f}, Y: {angle_y:.2f}, Z: {angle_z:.2f}")

        # Clear the plot
        ax.clear()

        # Update the plane model with the new orientation
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(np.radians(angle_x)), -np.sin(np.radians(angle_x))],
            [0, np.sin(np.radians(angle_x)), np.cos(np.radians(angle_x))]
        ])
        Ry = np.array([
            [np.cos(np.radians(angle_y)), 0, np.sin(np.radians(angle_y))],
            [0, 1, 0],
            [-np.sin(np.radians(angle_y)), 0, np.cos(np.radians(angle_y))]
        ])
        Rz = np.array([
            [np.cos(np.radians(angle_z)), -np.sin(np.radians(angle_z)), 0],
            [np.sin(np.radians(angle_z)), np.cos(np.radians(angle_z)), 0],
            [0, 0, 1]
        ])

        R = Rz @ Ry @ Rx  # Combined rotation matrix
        rotated_vertices = vertices @ R.T

        # Update faces with rotated vertices
        faces_rotated = [
            [rotated_vertices[0], rotated_vertices[1], rotated_vertices[3]],
            [rotated_vertices[0], rotated_vertices[2], rotated_vertices[3]],
            [rotated_vertices[0], rotated_vertices[1], rotated_vertices[4]],
            [rotated_vertices[0], rotated_vertices[2], rotated_vertices[4]],
            [rotated_vertices[1], rotated_vertices[2], rotated_vertices[3]],
            [rotated_vertices[1], rotated_vertices[2], rotated_vertices[4]]
        ]

        plane = Poly3DCollection(faces_rotated, alpha=0.5, edgecolors='k')
        ax.add_collection3d(plane)

        # Set plot limits and labels
        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([-2, 2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

    except Exception as e:
        print(f"Error reading gyro data: {e}")

if __name__ == "__main__":
    # Initialize plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plane model vertices
    vertices = np.array([
        [1, 0, 0],
        [-1, 0.5, 0],
        [-1, -0.5, 0],
        [0, 0, 0.5],
        [0, 0, -0.5]
    ])

    # Plane model faces
    faces = [
        [vertices[0], vertices[1], vertices[3]],
        [vertices[0], vertices[2], vertices[3]],
        [vertices[0], vertices[1], vertices[4]],
        [vertices[0], vertices[2], vertices[4]],
        [vertices[1], vertices[2], vertices[3]],
        [vertices[1], vertices[2], vertices[4]]
    ]

    plane = Poly3DCollection(faces, alpha=0.5, edgecolors='k')
    ax.add_collection3d(plane)

    ani = FuncAnimation(fig, update, interval=100)
    plt.tight_layout()
    plt.show()
