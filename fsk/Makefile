ALL_CFLAGS := -Wall -Wextra -Werror -std=c99 -D_XOPEN_SOURCE=700 -g $(CFLAGS)
OBJS := sender.o receiver.o pa_ringbuffer.o
DEPS := $(OBJS:.o=.d)

.PHONY: all
all: sender receiver

sender: sender.o pa_ringbuffer.o
	$(CC) $(ALL_CFLAGS) -o $@ $^ -lm -lportaudio

receiver: receiver.o pa_ringbuffer.o
	$(CC) $(ALL_CFLAGS) -o $@ $^ -lm -lportaudio -lfftw3f

-include $(DEPS)

%.o: %.c
	$(CC) $(ALL_CFLAGS) -MMD -o $@ -c $<

.PHONY: clean
clean:
	rm -f sender receiver $(OBJS) $(DEPS)
