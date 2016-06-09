all:
	g++ intruder_1.cpp -o image/intruder `pkg-config opencv --cflags --libs`
