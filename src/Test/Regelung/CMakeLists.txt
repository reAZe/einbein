add_subdirectory(Trajektorie)
add_subdirectory(Base2Tool)
add_subdirectory(Controller)
add_subdirectory(PDV)
add_subdirectory(Encoder)
add_subdirectory(PMotor)
add_subdirectory(IMU)
add_subdirectory(ZustBest)


add_executable(Controll mainControll.cpp CSControll.cpp Encoder/Encoder.cpp Encoder/TransitionBlockEncoder.cpp Base2Tool/VorKin.cpp 
		MotorModell/MotorModell.cpp MotorModell/I2DAC.cpp PMotor/PMotor.cpp Base2Tool/VorKin.cpp Trajektorie/Trajektorie.cpp
		PDV/PDV.cpp Trajektorie/constInput.cpp Controller/zControll.cpp Controller/Controller.cpp ZustBest/ZustBest.cpp IMU/IMU.cpp)
target_link_libraries(Controll eeros flink mpu9250)

