################################################################################
#                                    PROJECT                                   #
################################################################################
CONTIKI_PROJECT=tpt
################################################################################
#                                     FILES                                    #
################################################################################
CONTIKI=../../contiki_src/contiki

SRC_PATH=./src
INC_PATH=./include

SRC_FILES = $(notdir $(wildcard $(SRC_PATH)/*.c))

PROJECTDIRS += $(SRC_PATH)
PROJECTDIRS += $(INC_PATH)
PROJECT_SOURCEFILES += $(SRC_FILES)

BOARD_NAME = $(subst /,-,$(BOARD))
################################################################################
#                                     FLAGS                                    #
################################################################################
#linker optimizations
SMALL=1

ifeq ($(PREFIX),)
 PREFIX = aaaa::1/64
endif

ifeq ($(TARGET),exp5438)
  CFLAGS += -DEXP5438_DISABLE_UART=1
endif

CONTIKI_WITH_IPV6 = 1

CFLAGS += -DCM_CONF_AUTO_STROBES=1
CFLAGS+=-DPROJECT_CONF_H=\"project-conf.h\"

#TI_CC26XXWARE := ../../libs/cc26xxware/cc26xxware_2_20_06_14829
################################################################################
#                                    TARGETS                                   #
################################################################################
all: $(CONTIKI_PROJECT)
	cp  $(CONTIKI_PROJECT).bin $(CONTIKI_PROJECT)-$(BOARD_NAME).bin
################################################################################
#                                   INCLUDES                                   #
################################################################################
include $(CONTIKI)/Makefile.include
