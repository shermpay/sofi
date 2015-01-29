ALL_CFLAGS := -Wall -Wextra -Werror -std=c99 -D_XOPEN_SOURCE=700 -g $(CFLAGS)

.PHONY: all
all: sender

sender: sender.o
	$(CC) $(ALL_CFLAGS) -o $@ $^ -lm -lportaudio

%.o: %.c
	$(CC) $(ALL_CFLAGS) -o $@ -c $<

.PHONY: clean
clean:
	rm -f sender sender.o
