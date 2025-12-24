"""
  Imu Data: 
    - acc (x, y, z) m/s^2
    - gyro (x, y, z) rad/s
    - mag (x, y, z) uT

  GPS Data (NMEA 0183): 
    - $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0 0031*4F
      - format
      - UTC (172814.0)
      - Latitude (3723.46587704)
      - Direction (N)
      - Longitude (12202.26957864)
      - Direction (W)
      - GPS Quality 
        0: Fix not valid
        1: GPS fix
        2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
        3: Not applicable
        4: RTK Fixed, xFill
        5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX
        6: INS Dead reckoning
      ...
      - Orthometric height (m)
    - GPRMC,203522.00,A,5109.0262308,N,11401.8407342,W,0.004,133.4,130522,0.0,E,D*2B

    -- Latitude and Longitude

  Output frame @ t: 
    - orientation
      roll, pitch, yaw in radians
    - angular velocity 
      roll_rate, pitch_rate, yaw_rate in rad/s
    - linear acceleration (gravity-free, in body frame)
      a_forward, a_lateral, a_vertical in m/s2
    - position
      lat, lon, altitude
    - velocity
      speed and vertical speed in m/s 
"""




