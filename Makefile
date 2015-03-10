ALL_CFLAGS := -Wall -Wextra -Werror -std=c99 -I. -g $(CFLAGS)
BUILD ?= build
SOFINC_OBJS := $(addprefix $(BUILD)/, sofinc/sofinc.o)
LIBSOFI_OBJS := $(addprefix $(BUILD)/, libsofi/libsofi.o libsofi/pa_ringbuffer.o)
OBJS := $(SOFINC_OBJS) $(LIBSOFI_OBJS)
DEPS := $(OBJS:.o=.d)

dir_guard = @mkdir -p $(@D)

.PHONY: all
all: $(BUILD)/sofinc/sofinc

$(BUILD)/libsofi/libsofi.a: $(LIBSOFI_OBJS)
	$(dir_guard)
	$(AR) rcs $@ $^

$(BUILD)/sofinc/sofinc: $(SOFINC_OBJS) $(BUILD)/libsofi/libsofi.a
	$(dir_guard)
	$(CC) $(ALL_CFLAGS) -o $@ $^ -pthread -lm -lportaudio

$(BUILD)/%.o: %.c
	$(dir_guard)
	$(CC) $(ALL_CFLAGS) -MMD -o $@ -c $< -pthread

-include $(DEPS)

.PHONY: clean
clean:
	rm -f $(BUILD)/sofinc/sofinc $(BUILD)/libsofi/libsofi.a
	rm -f $(OBJS) $(DEPS)
	rmdir $(BUILD)/sofinc $(BUILD)/libsofi $(BUILD)
