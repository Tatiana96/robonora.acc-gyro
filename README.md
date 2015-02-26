# robonora.acc-gyro
Some tests with accelerometer and gyroscope (ADXL345, MPU6050).

Plan:

1. Get data from accelerometer sensor.
2. Get data from gyroscop sensor.
3. Find rotation axis-angle.
4. Use Kalman filter on gyroscope angle.
5. Use rotation matrix on gyroscope angle.
6. Profit!

TO-DO:

1. Find out IMU (Inertial Measurement Unit) of raw data from MPU6050 and ADXL345.
2. Build a theory about the data we need to get from rotation matrix.
3. Check out the theory.