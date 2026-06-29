# =============================================================================
# Makefile for DuoDX GUI  (MSYS2 / MinGW-w64)
#
#   make            Build duodx.exe (optimised, windowed, stripped)
#   make debug      Build with -g and a console attached for stdout/stderr
#   make clean      Remove build artefacts
#   make run        Build then launch
#
# Override the SDRplay API location if it is installed elsewhere:
#   make SDR_API="D:/SDRplay/API"
# =============================================================================

CC      := gcc
WINDRES := windres
TARGET  := duodx.exe
SRC     := duodx.c
RES_SRC := duodx.rc
RES_OBJ := duodx.res

# --- SDRplay API v3 location -------------------------------------------------
# Default install path; override on the command line if needed.
SDR_API ?= C:/Program Files/SDRplay/API
SDR_INC := $(SDR_API)/inc
SDR_LIB := $(SDR_API)/x64

# --- Flags -------------------------------------------------------------------
CFLAGS  := -O2 -Wall -mthreads -I"$(SDR_INC)"
LDFLAGS := -L"$(SDR_LIB)"
LIBS    := -lsdrplay_api -lcomctl32 -lgdi32 -lws2_32 -lwinmm
GUIFLAG := -mwindows

# -----------------------------------------------------------------------------
.PHONY: all debug clean run strip-target

all: $(TARGET) strip-target

$(RES_OBJ): $(RES_SRC) duodx.ico
	$(WINDRES) $< -O coff -o $@

$(TARGET): $(SRC) $(RES_OBJ)
	$(CC) $(CFLAGS) $(GUIFLAG) -o $@ $< $(RES_OBJ) $(LDFLAGS) $(LIBS)

# Strip only for release builds (the all target).
.PHONY: strip-target
strip-target: $(TARGET)
	strip $(TARGET)

# Debug build: keeps a console window so printf/stderr and the debugger work,
# and does NOT strip, so -g symbols survive.
debug: CFLAGS := -O0 -g -Wall -mthreads -I"$(SDR_INC)"
debug: GUIFLAG :=
debug: clean $(TARGET)

run: all
	./$(TARGET)

clean:
	-rm -f $(TARGET) $(RES_OBJ)
