all: alexa_voice_control/alexa_voice_control.py arduino_odometry/arduino_odometry.py arduino_motor/arduino_motor.py localino_server/localino_server.py sixaxis_publisher/sixaxis_publisher.py berryimu_publisher/berryimu_publisher.py
	cd .. && catkin_make
