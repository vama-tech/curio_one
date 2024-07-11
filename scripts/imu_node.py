import smbus2

# I2C bus number (replace with the actual bus number for your Jetson)
bus_number = 1

# I2C address of the ICM-20948 (check datasheet or confirm via jumper settings)
i2c_address = 0x69

def read_sensor_data():
    """Reads sensor data from the ICM-20948 IMU.

    Returns:
        tuple: A tuple containing the following sensor readings:
            - accel_x (int): Accelerometer X-axis value (16 bits)
            - accel_y (int): Accelerometer Y-axis value (16 bits)
            - accel_z (int): Accelerometer Z-axis value (16 bits)
            - gyro_x (int): Gyroscope X-axis value (16 bits)
            - gyro_y (int): Gyroscope Y-axis value (16 bits)
            - gyro_z (int): Gyroscope Z-axis value (16 bits)
            - mag_x (int): Magnetometer X-axis value (16 bits)
            - mag_y (int): Magnetometer Y-axis value (16 bits)
            - mag_z (int): Magnetometer Z-axis value (16 bits)
            - temp (int): Temperature sensor value (16 bits)
    """

    with smbus2.SMBus(bus_number) as bus:
        # Read accelerometer data (16 bits each for X, Y, Z)
        accel_data = bus.read_i2c_block_data(i2c_address, 0x3B, 6)
        accel_x = (accel_data[1] << 8) | accel_data[0]
        accel_y = (accel_data[3] << 8) | accel_data[2]
        accel_z = (accel_data[5] << 8) | accel_data[4]

        # Read gyroscope data (16 bits each for X, Y, Z)
        gyro_data = bus.read_i2c_block_data(i2c_address, 0x43, 6)
        gyro_x = (gyro_data[1] << 8) | gyro_data[0]
        gyro_y = (gyro_data[3] << 8) | gyro_data[2]
        gyro_z = (gyro_data[5] << 8) | gyro_data[4]

        # Read magnetometer data (16 bits each for X, Y, Z)
        mag_data = bus.read_i2c_block_data(i2c_address, 0x49, 6)
        mag_x = (mag_data[1] << 8) | mag_data[0]
        mag_y = (mag_data[3] << 8) | mag_data[2]
        mag_z = (mag_data[5] << 8) | mag_data[4]

        # Read temperature data (16 bits)
        temp_data = bus.read_i2c_block_data(i2c_address, 0x78, 2)
        temp = (temp_data[1] << 8) | temp_data[0]

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temp

if __name__ == "__main__":
    while True:
        # Read sensor data
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temp = read_sensor_data()

        # Print sensor readings (modify this section for your specific use case)
        print("Accelerometer (g): X=%.2f, Y=%.2f, Z=%.2f" % (accel_x / 16384.0, accel_y / 16384.0, accel_z / 16384.0))
        print("Gyroscope (deg/s): X=%.2f, Y=%.2f, Z=%.2f" % (gyro_x / 14.5, gyro_y / 14.5))
