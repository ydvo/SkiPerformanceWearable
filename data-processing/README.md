Data Pipeline: 
1. Three Devices (Left boot, right boot, pelvic)
  Left and Right Boot: 
    IMU, GPS, Pressure sensor

  Pelvic: 
    IMU, GPS

2. Device -> IOS App via BLE (Raw Data)
  - timestamp
  - acc
  - gyro
  - mag
  - lat
  - lon
  - speed (if available)
  - altitude

3. IOS App flush raw data to Cloud Storage (S3 ex.)
4. IOS App do subjective real-time report (this would be of low accuracy, thus show qualitative metrics, like relative effort, speed etc.)
5. At the end of the run, send a request to process raw data with higher accuracy in the cloud
  - this would include filtering, feature extraction, and data aggregation across runs