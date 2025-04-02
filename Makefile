# Email: shiraba01@gmail.com

# Compiler and Flags
CXX = g++
CXXFLAGS = -std=c++11 -Wall

# Source and Header Files
SRC = Graph.cpp Algorithms.cpp Queue.cpp
MAIN = main.cpp
EXEC = main

# Object files for main build
OBJ = $(SRC:.cpp=.o)
MAIN_OBJ = $(MAIN:.cpp=.o)

# Test Files 
TEST_SRC = test_algorithms.cpp test_graph.cpp test_queue.cpp
TEST_OBJ = $(TEST_SRC:.cpp=.o)
TEST_EXEC = test_program

# Default target: Build main program
all: $(EXEC)

# Compile main executable from object files
$(EXEC): $(OBJ) $(MAIN_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile source files to object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Run the main program
Main: $(EXEC)
	./$(EXEC)

# Run unit tests 
test: $(TEST_SRC) $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TEST_EXEC) $(TEST_SRC) $(SRC)
	./$(TEST_EXEC)

# Valgrind memory check
valgrind: $(EXEC)
	valgrind --leak-check=full ./$(EXEC)

# Clean up all built files
clean:
	rm -f $(EXEC) $(TEST_EXEC) *.o
