add_executable(IMU mainIMU.cpp CSIMU.cpp IMU.cpp)	    
target_link_libraries(IMU mpu9250 eeros)