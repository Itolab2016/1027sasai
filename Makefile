test_time:test_time.cpp tsukuba_time.h tsukuba_time.cpp Makefile
	g++ -o test_time test_time.cpp  tsukuba_time.cpp

test_tkk:test_tkk.cpp tsukuba_tkk.h tsukuba_tkk.cpp Makefile
	g++ -o test_tkk test_tkk.cpp  tsukuba_tkk.cpp

test_urg:test_urg.cpp tsukuba_urg.h tsukuba_urg.cpp  Makefile
	g++ -o test_urg test_urg.cpp tsukuba_urg.cpp Connection_information.cpp `pkg-config --libs opencv` `urg_cpp-config --cflags` `urg_cpp-config --libs`

test:test.cpp tsukuba_time.h tsukuba_time.cpp tsukuba_tkk.h tsukuba_tkk.cpp Makefile
	g++ -o test test.cpp tsukuba_time.cpp tsukuba_tkk.cpp

motor_test:motor_test.cpp motor.h motor.cpp joystick.h joystick.cpp tsukuba_time.h tsukuba_time.cpp Makefile
	g++ -o motor_test motor_test.cpp motor.cpp joystick.cpp tsukuba_time.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`


camera_test:camera_test.cpp camera.h camera.cpp Makefile
	g++ -o camera_test camera_test.cpp camera.cpp  `pkg-config --cflags opencv` `pkg-config --libs opencv`

joystick_test:joystick_test.cpp joystick.h joystick.cpp Makefile
	g++ -o joystick_test joystick_test.cpp joystick.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`

main:main.cpp camera.h camera.cpp joystick.h joystick.cpp motor.h motor.cpp tsukuba_time.h tsukuba_time.cpp tsukuba_tkk.h tsukuba_tkk.cpp tsukuba_urg.h tsukuba_urg.cpp Makefile
	g++ -o main main.cpp camera.cpp motor.cpp joystick.cpp tsukuba_time.cpp tsukuba_tkk.cpp tsukuba_urg.cpp Connection_information.cpp `pkg-config --libs opencv` `urg_cpp-config --cflags` `urg_cpp-config --libs` `pkg-config --cflags opencv` `pkg-config --libs opencv`

test_tkk:test_tkk.cpp tsukuba_tkk.h tsukuba_tkk.cpp tsukuba_time.cpp tsukuba_time.h Makefile
	g++ -o test_tkk test_tkk.cpp tsukuba_tkk.cpp tsukuba_time.cpp
