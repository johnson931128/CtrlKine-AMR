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

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET) $(LDFLAGS) $(LDLIBS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rmdir /S /Q $(BUILD_DIR)