ALL_CFLAGS := -Wall -Wextra -Werror -std=c99 -D_XOPEN_SOURCE=700 -g $(CFLAGS)

.PHONY: all
all: sender

receiver: receiver.o pa_ringbuffer.o
	$(CC) $(ALL_CFLAGS) -o $@ $^ -lm -lportaudio

sender: sender.o pa_ringbuffer.o
	$(CC) $(ALL_CFLAGS) -o $@ $^ -lm -lportaudio

%.o: %.c
	$(CC) $(ALL_CFLAGS) -o $@ -c $<

.PHONY: clean
clean:
	rm -f sender sender.o receiver receiver.o pa_ringbuffer.o
