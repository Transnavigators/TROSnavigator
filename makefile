all: alexa_voice_control/alexa_voice_control.py arduino_control/arduino_control.py arduino_publisher/arduino_publisher.py localino_server/localino_server.py
	cd .. && catkin_make
