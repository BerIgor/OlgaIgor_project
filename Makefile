FLAGS = -std=c++11
INCLUDE = -I/usr/local/include
LIBDIR = -L/usr/local/lib
LIBS = -lompl -lboost_system

PLANNER = planner
PLANNER_OBJS = src/planner.o

planner.o: src/planner.cpp
		$(CC) -c $(FLAGS) src/planner.cpp $(LIBS)

$(PLANNER) : $(PLANNER_OBJS) 
		$(CC) $(FLAGS) $(INCLUDE) $(PLANNER_OBJS) $(LIBDIR) $(LIBS) -o $@

all: $(PLANNER)

clean:
	rm -f $(PLANNER_OBJS)
