build-debug:
	cmake --preset debug
	cmake --build --preset debug

format:
	find $(CURDIR)/homeworks -name "build" -prune -o -name "*.cpp" -print -o -name "*.hpp" -print | xargs clang-format -i
	find $(CURDIR)/homeworks -name "build" -prune -o -name "CMakeLists.txt" -print | xargs cmake-format -i

lint:
	run-clang-tidy -p build/debug -config-file=.devcontainer/.clang-tidy ./homeworks

test:
	ctest --test-dir build/debug --output-on-failure