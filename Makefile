ALL_CFLAGS := -Wall -Wextra -Werror -std=c99 -g $(CFLAGS)
BUILD ?= build
OBJS := $(addprefix $(BUILD)/, sofinc.o pa_ringbuffer.o)
DEPS := $(OBJS:.o=.d)

.PHONY: all
all: $(BUILD)/sofinc

dir_guard = @mkdir -p $(@D)

$(BUILD)/sofinc: $(BUILD)/sofinc.o $(BUILD)/pa_ringbuffer.o
	$(dir_guard)
	$(CC) $(ALL_CFLAGS) -o $@ $^ -pthread -lm -lportaudio

$(BUILD)/%.o: %.c
	$(dir_guard)
	$(CC) $(ALL_CFLAGS) -MMD -o $@ -c $< -pthread

-include $(DEPS)

.PHONY: clean
clean:
	rm -f $(BUILD)/sofinc $(OBJS) $(DEPS)
