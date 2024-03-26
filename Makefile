RM = /bin/rm
MV = /bin/mv
CP = /bin/cp
PYTHON = python3
MKDIR = mkdir
GCOV = gcov
LCOV = lcov
GENHTML = genhtml
TAR = gtar

# OS detection
ifeq '$(findstring ;,$(PATH))' ';'
    detected_OS := Windows
else
    detected_OS := $(shell uname 2>/dev/null || echo Unknown)
    detected_OS := $(patsubst CYGWIN%,Cygwin,$(detected_OS))
    detected_OS := $(patsubst MSYS%,MSYS,$(detected_OS))
    detected_OS := $(patsubst MINGW%,MSYS,$(detected_OS))
endif

# Workaround for Mac
ifneq ("$(wildcard /opt/homebrew/bin/g++-13)","")
export CC=/opt/homebrew/bin/gcc-13
export CXX=/opt/homebrew/bin/g++-13
GCOV = /opt/homebrew/bin/gcov-13
endif

ifeq ($(detected_OS),Darwin)  # Mac OS X
    OPEN = open
else
    OPEN = echo "Created:"
endif

CMAKE=cmake
CMAKE_OPTS=-DCMAKE_C_COMPILER=$(CC) -DCMAKE_CXX_COMPILER=$(CXX)

SRC_CPP=src/main/cpp
TEST_CPP=src/test/cpp
PROJ_DIR=$(PWD)
BUILD_DIR=$(PROJ_DIR)/build
DIST_DIR=$(PROJ_DIR)/dist
TEST_BIN_DIR=$(BUILD_DIR)/test
SOLVER_BIN=hydrax
TEST_EXEC=$(TEST_BIN_DIR)/$(SOLVER_BIN)_test
COV_CPP_DIR=coverage/cpp
COV_CPP=$(COV_CPP_DIR)/lcov.info
COV_PY_DIR=coverage/py
COV_PY=$(COV_PY_DIR)/lcov.info
COV_HTML=coverage/html
COV_MERGED=coverage/lcov.info

SRC_PY=src/main/python
TEST_PY=src/test/python
STUB_PY=src/stubs

export PYTHONPATH=$(PWD)/$(SRC_PY)
export MYPYPATH=$(PWD)/$(STUB_PY)

build:
	cd $(SRC_CPP) && $(CMAKE) -DCMAKE_BUILD_TYPE=Release -S . -B "$(BUILD_DIR)/Release" $(CMAKE_OPTS)
	cd $(SRC_CPP) && $(CMAKE) --build "$(BUILD_DIR)/Release"
	@echo "Created: build/Release/$(SOLVER_BIN)"

build-debug:
	cd $(SRC_CPP) && $(CMAKE) -DCMAKE_BUILD_TYPE=Debug -S . -B "$(BUILD_DIR)/Debug" $(CMAKE_OPTS)
	cd $(SRC_CPP) && $(CMAKE) --build "$(BUILD_DIR)/Debug"
	@echo "Created: build/Debug/$(SOLVER_BIN)"

test: test-cpp

test-cpp:
# check style
	find "$(SRC_CPP)" "$(TEST_CPP)" -regex '.*[.][ch]pp' | xargs clang-format --dry-run -Werror
# run tests
	@echo "GTEST_FILTER: $(GTEST_FILTER)"
	cd $(SRC_CPP) && $(CMAKE) -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTS=ON -S . -B "$(BUILD_DIR)/UT" $(CMAKE_OPTS)
	cd $(SRC_CPP) && $(CMAKE) --build "$(BUILD_DIR)/UT"
	"$(TEST_EXEC)" --output-on-failure $(GTEST_OPTS)

test-cpp-cov: test-cpp
	$(MKDIR) -p "$(COV_CPP_DIR)"
	$(LCOV) -q --gcov-tool $(GCOV) -d "$(TEST_BIN_DIR)" -c -o "$(COV_CPP)"
	$(LCOV) -q -r "${COV_CPP}" "*/include/*" "*.h" "*/test/*" "*/build*/*" "*/external/*" -o "${COV_CPP}"
	$(LCOV) -a $(COV_CPP) -o $(COV_MERGED)

test-py:
# check style
	autopep8 --recursive --diff --exit-code "$(SRC_PY)" "$(TEST_PY)"
# check types
	mypy "$(SRC_PY)"
# run tests with coverage
	$(MKDIR) -p "$(COV_PY_DIR)"
	$(PYTHON) -m pytest -x --cov="$(SRC_PY)" --cov-report=lcov:$(COV_PY) $(PYTEST_OPTS) $(TEST_PY)

coverage: test-cpp-cov test-py
	$(LCOV) --add-tracefile $(COV_PY) -a $(COV_CPP) -o $(COV_MERGED)

coverage-html: coverage
	$(GENHTML) -o $(COV_HTML) $(COV_MERGED)
	@$(OPEN) $(COV_HTML)/index.html

clean:
	@echo "Cleaning..."
	@$(RM) -rf build/* coverage/*
	@echo "Cleaning done."

lab:
	jupyter-lab

.PHONY: build build-debug test test-cpp test-cpp-cov test-py coverage coverage-html clean lab
