CXX = g++
# 更新 Include 路徑至之前成功的 SFML-3.0.0 資料夾
CXXFLAGS = -std=c++17 -Wall -Iinclude -I"C:/lib/SFML-3.0.0/include"

# 更新 Lib 路徑至之前成功的 SFML-3.0.0 資料夾
LDFLAGS = -L"C:/lib/SFML-3.0.0/lib"
LDLIBS = -lsfml-graphics -lsfml-window -lsfml-system

SRC_DIR = src
BUILD_DIR = build

SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(SRCS:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)

TARGET = $(BUILD_DIR)/CtrlKine-AMR.exe

TEST_BUILD_DIR = $(BUILD_DIR)/tests
TEST_MAPPER_TARGET = $(TEST_BUILD_DIR)/CoordinateMapperTests.exe
TEST_MAP_TARGET = $(TEST_BUILD_DIR)/MapDataTests.exe
TEST_FILE_TARGET = $(TEST_BUILD_DIR)/MapDataFileTests.exe
TEST_PATH_TARGET = $(TEST_BUILD_DIR)/PathPlannerTests.exe
TEST_EXECUTION_TARGET = $(TEST_BUILD_DIR)/PathExecutionTests.exe
TEST_RUNTIME_TARGET = $(TEST_BUILD_DIR)/SimulatorRuntimeTests.exe
TEST_TARGETS = $(TEST_MAPPER_TARGET) $(TEST_MAP_TARGET) $(TEST_FILE_TARGET) $(TEST_PATH_TARGET) $(TEST_EXECUTION_TARGET) $(TEST_RUNTIME_TARGET)

all: $(TARGET)

test: $(TEST_TARGETS)
	@status=0; \
	$(TEST_MAPPER_TARGET) || status=1; \
	$(TEST_MAP_TARGET) || status=1; \
	$(TEST_FILE_TARGET) || status=1; \
	$(TEST_PATH_TARGET) || status=1; \
	$(TEST_EXECUTION_TARGET) || status=1; \
	$(TEST_RUNTIME_TARGET) || status=1; \
	exit $$status

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET) $(LDFLAGS) $(LDLIBS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TEST_BUILD_DIR)/CoordinateMapperTests.o: tests/CoordinateMapperTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/MapDataTests.o: tests/MapDataTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/MapDataFileTests.o: tests/MapDataFileTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/PathPlannerTests.o: tests/PathPlannerTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/PathExecutionTests.o: tests/PathExecutionTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/SimulatorRuntimeTests.o: tests/SimulatorRuntimeTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/MapData.o: src/MapData.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TEST_MAPPER_TARGET): $(TEST_BUILD_DIR)/CoordinateMapperTests.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_MAP_TARGET): $(TEST_BUILD_DIR)/MapDataTests.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_FILE_TARGET): $(TEST_BUILD_DIR)/MapDataFileTests.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_PATH_TARGET): $(TEST_BUILD_DIR)/PathPlannerTests.o $(TEST_BUILD_DIR)/PathPlanner.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_EXECUTION_TARGET): $(TEST_BUILD_DIR)/PathExecutionTests.o $(TEST_BUILD_DIR)/PathExecution.o $(TEST_BUILD_DIR)/AMR.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_RUNTIME_TARGET): $(TEST_BUILD_DIR)/SimulatorRuntimeTests.o $(BUILD_DIR)/Simulator.o $(BUILD_DIR)/Environment.o $(BUILD_DIR)/MapValidator.o $(TEST_BUILD_DIR)/PathExecution.o $(TEST_BUILD_DIR)/PathPlanner.o $(TEST_BUILD_DIR)/MapData.o $(TEST_BUILD_DIR)/AMR.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_BUILD_DIR)/PathPlanner.o: src/PathPlanner.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TEST_BUILD_DIR)/PathExecution.o: src/PathExecution.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TEST_BUILD_DIR)/AMR.o: src/AMR.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
