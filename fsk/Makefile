ALL_CFLAGS := -Wall -Wextra -Werror -std=c99 -D_XOPEN_SOURCE=700 -g $(CFLAGS)

.PHONY: all
all: sender receiver

sender: sender.o fsk.h
	$(CC) $(ALL_CFLAGS) -o $@ $^ -lm -lportaudio

receiver: receiver.o pa_ringbuffer.o fsk.h
	$(CC) $(ALL_CFLAGS) -o $@ $^ -lm -lportaudio -lfftw3f

%.o: %.c
	$(CC) $(ALL_CFLAGS) -o $@ -c $<

.PHONY: clean
clean:
	rm -f sender sender.o receiver receiver.o pa_ringbuffer.o
