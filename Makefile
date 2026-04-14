BUILD_DIR := build/build/Release

.PHONY: all clean rebuild

all:
	cmake --preset conan-release
	cmake --build $(BUILD_DIR)

clean:
	cmake --build $(BUILD_DIR) --target clean

rebuild: clean all
