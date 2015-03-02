ALL_CFLAGS := -Wall -Wextra -Werror -std=c99 -g $(CFLAGS)
OBJS := sofinc.o pa_ringbuffer.o
DEPS := $(OBJS:.o=.d)

.PHONY: all
all: sofinc

sofinc: sofinc.o pa_ringbuffer.o
	$(CC) $(ALL_CFLAGS) -o $@ $^ -pthread -lm -lportaudio

%.o: %.c
	$(CC) $(ALL_CFLAGS) -MMD -o $@ -c $< -pthread

-include $(DEPS)

.PHONY: clean
clean:
	rm -f sofinc $(OBJS) $(DEPS)
