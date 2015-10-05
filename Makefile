#
MYCFLAGS := -fpermissive -w
MYCFLAGS += -D_ATS_CCOMP_HEADER_NONE_
MYCFLAGS += -D_ATS_CCOMP_PRELUDE_NONE_
MYCFLAGS += -D_ATS_CCOMP_PRELUDE_USER_='"arduino/H/pats_ccomp.h"'
# MYCFLAGS += -D_ATS_CCOMP_PRELUDE_USER2='"arduino/H/pats_ccomp2.h"'
MYCFLAGS += -D_ATS_CCOMP_EXCEPTION_NONE_
MYCFLAGS += -D_ATSTYPE_VAR_SIZE=1024
# Declare 64 bit compilation (results in new errors)
# MYCFLAGS += -D__x86_64__
# MYCFLAGS += -D__LP64__
MYCFLAGS += -I$(PATSHOME) -I$(PATSHOME)/ccomp/runtime -I${PATSHOMERELOC}/contrib
# -I/usr/include
#
CFLAGS_STD = $(MYCFLAGS)
CXXFLAGS_STD = $(MYCFLAGS)
#
BOARD_TAG = uno
ARDUINO_LIBS =
include ./../../Arduino-Makefile/Arduino.mk
#
######

MAKE=make

######
#
# Please make sure Arduino.mk is available
#
Arduino_mk=./../../Arduino-Makefile/Arduino.mk
Arduino_mk_src=https://github.com/sudar/Arduino-Makefile.git
#
.INTERMEDIATE: $(Arduino_mk)
#
$(Arduino_mk): ; git clone $(Arduino_mk_src) ./../../Arduino-Makefile
#
######

cleanall:: ; $(MAKE) -f Makefile_ats cleanall

######
#
###### end of [Makefile] ######
