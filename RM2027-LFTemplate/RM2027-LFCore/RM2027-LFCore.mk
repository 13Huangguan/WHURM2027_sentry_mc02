#######################################
# LuojiaFox RM2027 LFCore makefile
#######################################

# LFCore is C++; generated HAL and RTOS C sources remain in the top-level Makefile.

# Optimize control code while retaining finite/NaN semantics. Override LFCORE_OPT=-Og for stepping.
LFCORE_OPT ?= -O3
LFCORE_PROFILE ?= 0
$(BUILD_DIR)/$(CORE_DIR)/%.o: CPPFLAGS += $(LFCORE_OPT) -fno-math-errno -DLFCORE_PROFILE=$(LFCORE_PROFILE)

CPP_SOURCES += \
$(wildcard $(CORE_DIR)/1_Middleware/*/*.cpp) \
$(wildcard $(CORE_DIR)/1_Middleware/*/*/*.cpp) \
$(wildcard $(CORE_DIR)/2_Device/*/*.cpp) \
$(wildcard $(CORE_DIR)/4_Application/*/*.cpp) \
$(wildcard $(CORE_DIR)/3_LuojiaFox/*.cpp) \
$(wildcard $(CORE_DIR)/core_template.cpp)

LFCORE_OBJECTS := $(addprefix $(BUILD_DIR)/,$(filter $(CORE_DIR)/%,$(CPP_SOURCES:.cpp=.o)))
$(LFCORE_OBJECTS): $(CORE_DIR)/RM2027-LFCore.mk Core.mk

# 获取所有子目录（用于头文件包含路径）
MIDDLEWARE_SUBDIRS := \
$(patsubst %/,%,$(wildcard $(CORE_DIR)/1_Middleware/*/)) \
$(patsubst %/,%,$(wildcard $(CORE_DIR)/1_Middleware/*/*/))

DEVICE_SUBDIRS := \
$(wildcard $(CORE_DIR)/2_Device/*)

APP_SUBDIRS := \
$(wildcard $(CORE_DIR)/4_Application/*)

LUOJIAFOX_SUBDIRS := \
$(wildcard $(CORE_DIR)/3_LuojiaFox)

C_INCLUDES += \
-I$(CORE_DIR) \
$(addprefix -I,$(MIDDLEWARE_SUBDIRS)) \
$(addprefix -I,$(DEVICE_SUBDIRS)) \
$(addprefix -I,$(APP_SUBDIRS)) \
$(addprefix -I,$(LUOJIAFOX_SUBDIRS))
