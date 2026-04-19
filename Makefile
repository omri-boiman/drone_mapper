BUILD_DIR := build

.PHONY: all clean rebuild

all:
	cmake -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=Release
	cmake --build $(BUILD_DIR)

clean:
	cmake --build $(BUILD_DIR) --target clean

rebuild:
	rm -rf $(BUILD_DIR)
	$(MAKE) all
