# Makefile for DuoDX
# Targets MinGW-w64 (MSYS2) on Windows.
#
# Usage:
#   make              - build duodx.exe (release)
#   make debug        - build with debug symbols, no optimisation
#   make clean        - remove build output

# ---------------------------------------------------------------------------
# SDRplay API paths - adjust if your installation differs
# ---------------------------------------------------------------------------
SDRPLAY_INC = C:/Program Files/SDRplay/API/inc
SDRPLAY_LIB = C:/Program Files/SDRplay/API/x64

# ---------------------------------------------------------------------------
# Toolchain
# ---------------------------------------------------------------------------
CC      = gcc
TARGET  = duodx.exe
SRC     = duodx.c

# ---------------------------------------------------------------------------
# Flags
# ---------------------------------------------------------------------------
CFLAGS_COMMON = -Wall -Wextra -mthreads \
                -I"$(SDRPLAY_INC)"

CFLAGS_RELEASE = $(CFLAGS_COMMON) -O2
CFLAGS_DEBUG   = $(CFLAGS_COMMON) -O0 -g -DDEBUG

LDFLAGS = -L"$(SDRPLAY_LIB)" \
          -lsdrplay_api \
          -lwinmm \
          -lws2_32

# ---------------------------------------------------------------------------
# Targets
# ---------------------------------------------------------------------------
.PHONY: all debug clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS_RELEASE) -o $@ $< $(LDFLAGS)
	@echo Built $@ [release]

debug: $(SRC)
	$(CC) $(CFLAGS_DEBUG) -o $(TARGET) $< $(LDFLAGS)
	@echo Built $(TARGET) [debug]

clean:
	del /Q $(TARGET) 2>NUL || true
