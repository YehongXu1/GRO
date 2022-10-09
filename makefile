CXX=g++ -std=c++17 -I
OPT=-O3

hello: model.o tools.o traffic.o KSP.o baseline.o main.o -lboost_system -lpthread -lboost_thread
	$(CXX) -g -o hello *.o -lboost_system -lpthread -lboost_thread


model.o:model/*.cpp
	$(CXX) -g -c $(OPT) model/*.cpp
tools.o:tools/*.cpp
	$(CXX) -g -c $(OPT) tools/*.cpp
traffic.o:traffic/*.cpp
	$(CXX) -g -c $(OPT) traffic/*.cpp
KSP.o:KSP/*.cpp
	$(CXX) -g -c $(OPT) KSP/*.cpp
baseline.o:baseline/*.cpp
	$(CXX) -g -c $(OPT) baseline/*.cpp
main.o:main.cpp
	$(CXX) -g -c $(OPT) main.cpp

clean:
	rm *.o
	rm hello
