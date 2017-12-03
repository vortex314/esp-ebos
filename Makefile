#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := esp-ebos
TTY ?= USB0
SERIAL_PORT ?= /dev/tty$(TTY)
SERIAL_BAUD = 115200
ESPBAUD = 921600
IDF_PATH ?= /home/lieven/esp/esp-idf


include $(IDF_PATH)/make/project.mk

term:
	rm -f $(TTY)_minicom.log
	minicom -D $(SERIAL_PORT) -b $(SERIAL_BAUD) -C $(TTY)_minicom.log



