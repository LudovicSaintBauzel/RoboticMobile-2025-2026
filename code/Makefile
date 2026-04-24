ROOT := $(CURDIR)
SRC_DIR := code-robmob
TARGET_DIR := target

CC ?= cc
CXX ?= c++
CFLAGS ?= -O2 -Wall -Wextra -std=c11
CXXFLAGS ?= -O2 -Wall -Wextra -std=c++20
LDFLAGS ?=
RPI ?= 0

PTHREAD_LIB := -pthread
WIRINGPI_LIB ?= -lwiringPi

ifeq ($(RPI),1)
EXTRA_CFLAGS := -DRPI
EXTRA_CXXFLAGS := -DRPI
RUST_FEATURE_ARGS := --no-default-features --features rpi
ASSERVPI_C_LIBS := $(PTHREAD_LIB) $(WIRINGPI_LIB)
ASSERVPI_CPP_LIBS := $(PTHREAD_LIB) $(WIRINGPI_LIB)
else
EXTRA_CFLAGS :=
EXTRA_CXXFLAGS :=
RUST_FEATURE_ARGS :=
ASSERVPI_C_LIBS := $(PTHREAD_LIB)
ASSERVPI_CPP_LIBS := $(PTHREAD_LIB)
endif

BIN_C_VIRTUAL := $(TARGET_DIR)/encoder_asservpi_c
BIN_C_WIRING := $(TARGET_DIR)/encoder_wiringpi_c $(TARGET_DIR)/encoder $(TARGET_DIR)/moteur
BIN_CPP_VIRTUAL := $(TARGET_DIR)/encoder_asservpi_cpp
BIN_CPP_WIRING := $(TARGET_DIR)/encoder_wiringpi_cpp
BIN_RUST := $(TARGET_DIR)/encoder_asservpi_rs

ALL_BINS := $(BIN_C_VIRTUAL) $(BIN_C_WIRING) $(BIN_CPP_VIRTUAL) $(BIN_CPP_WIRING) $(BIN_RUST)

.PHONY: all c cpp rust rpi clean help

all: $(ALL_BINS)

c: $(BIN_C_VIRTUAL) $(BIN_C_WIRING)

cpp: $(BIN_CPP_VIRTUAL) $(BIN_CPP_WIRING)

rust: $(BIN_RUST)

rpi:
	$(MAKE) RPI=1 all

$(TARGET_DIR):
	mkdir -p $(TARGET_DIR)

$(TARGET_DIR)/encoder_asservpi_c: $(SRC_DIR)/encoder_asservpi.c | $(TARGET_DIR)
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $< -o $@ $(ASSERVPI_C_LIBS) $(LDFLAGS)

$(TARGET_DIR)/encoder_wiringpi_c: $(SRC_DIR)/encoder_wiringpi.c | $(TARGET_DIR)
	$(CC) $(CFLAGS) $< -o $@ $(PTHREAD_LIB) $(WIRINGPI_LIB) $(LDFLAGS)

$(TARGET_DIR)/encoder: $(SRC_DIR)/encoder.c | $(TARGET_DIR)
	$(CC) $(CFLAGS) $< -o $@ $(PTHREAD_LIB) $(WIRINGPI_LIB) $(LDFLAGS)

$(TARGET_DIR)/moteur: $(SRC_DIR)/moteur.c | $(TARGET_DIR)
	$(CC) $(CFLAGS) $< -o $@ $(WIRINGPI_LIB) $(LDFLAGS)

$(TARGET_DIR)/encoder_asservpi_cpp: $(SRC_DIR)/encoder_asservpi.cpp | $(TARGET_DIR)
	$(CXX) $(CXXFLAGS) $(EXTRA_CXXFLAGS) $< -o $@ $(ASSERVPI_CPP_LIBS) $(LDFLAGS)

$(TARGET_DIR)/encoder_wiringpi_cpp: $(SRC_DIR)/encoder_wiringpi.cpp | $(TARGET_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(PTHREAD_LIB) $(WIRINGPI_LIB) $(LDFLAGS)

$(TARGET_DIR)/encoder_asservpi_rs: $(SRC_DIR)/Cargo.toml $(SRC_DIR)/encoder_asservpi.rs | $(TARGET_DIR)
	CARGO_TARGET_DIR="$(ROOT)/$(TARGET_DIR)" cargo build --manifest-path "$(SRC_DIR)/Cargo.toml" --release --bin encoder_asservpi $(RUST_FEATURE_ARGS)
	cp "$(TARGET_DIR)/release/encoder_asservpi" "$@"

$(TARGET_DIR)/asserv_rpicam_rs: $(SRC_DIR)/Cargo.toml $(SRC_DIR)/asserv_rpicam.rs | $(TARGET_DIR)
	CARGO_TARGET_DIR="$(ROOT)/$(TARGET_DIR)" cargo build --manifest-path "$(SRC_DIR)/Cargo.toml" --release --bin asserv_rpicam $(RUST_FEATURE_ARGS)
	cp "$(TARGET_DIR)/release/asserv_rpicam" "$@"

clean:
	rm -f $(ALL_BINS)

help:
	@echo "Cibles disponibles :"
	@echo "  make            : compile tous les executables dans $(TARGET_DIR)/"
	@echo "  make c          : compile les programmes C"
	@echo "  make cpp        : compile les programmes C++"
	@echo "  make rust       : compile le binaire Rust (copie vers $(TARGET_DIR)/)"
	@echo "  make rpi        : equivalent a make RPI=1 all"
	@echo "  make clean      : supprime les executables generes"
	@echo "Variables utiles :"
	@echo "  RPI=1           : active les options Raspberry Pi (-DRPI + features Rust rpi)"
	@echo "  WIRINGPI_LIB='-lwiringPi' (defaut, surcharge possible)"
	@echo "  CFLAGS / CXXFLAGS / LDFLAGS"
