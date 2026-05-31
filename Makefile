SRC_DIR := homeworks

FIND_CPP := find $(SRC_DIR) -type f \( -name "*.cpp" -o -name "*.hpp" \) \
	-not -path "*/build/*" \
	-not -path "*/third_party/*"

FIND_CMAKE := find $(SRC_DIR) -name "CMakeLists.txt" \
	-not -path "*/build/*" \
	-not -path "*/third_party/*"

build-debug:
	cmake --preset debug
	cmake --build --preset debug

build-release:
	cmake --preset release
	cmake --build --preset release

build-relwithdebinfo:
	cmake --preset relwithdebinfo
	cmake --build --preset relwithdebinfo

format:
	$(FIND_CPP) -exec clang-format -i {} \;
	$(FIND_CMAKE) -exec cmake-format -i {} +

lint:
	run-clang-tidy -p build/debug ./$(SRC_DIR)

test:
	ctest --test-dir build/debug --output-on-failure