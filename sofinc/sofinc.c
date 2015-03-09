#include <ctype.h>
#include <inttypes.h>

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include "sofi.h"

static void *sender_loop(void *arg)
{
	struct sofi_packet packet;

	(void)arg;

	for (;;) {
		packet.len = fread(packet.payload, 1, sizeof(packet.payload),
				   stdin);
		if (packet.len == 0)
			break;
		sofi_send(&packet);
	}
	if (ferror(stdin) && errno != EINTR) {
		perror("fread");
		return (void *)-1;
	}
	return (void *)0;
}

static void *receiver_loop(void *arg)
{
	struct sofi_packet packet;

	(void)arg;

	for (;;) {
		sofi_recv(&packet);
		fwrite(packet.payload, 1, packet.len, stdout);
		fflush(stdout);
		if (ferror(stdout)) {
			perror("fflush");
			return (void *)-1;
		}
	}
	return (void *)0;
}

int main(void)
{
	pthread_t sender_thread, receiver_thread;
	int ret;
	int status = EXIT_SUCCESS;

	ret = sofi_init(true, true);
	if (ret)
		return EXIT_FAILURE;

	ret = pthread_create(&sender_thread, NULL, sender_loop, NULL);
	if (ret) {
		errno = ret;
		perror("pthread_create");
		status = EXIT_FAILURE;
		goto out;
	}

	ret = pthread_create(&receiver_thread, NULL, receiver_loop, NULL);
	if (ret) {
		pthread_cancel(sender_thread);
		pthread_join(sender_thread, NULL);
		errno = ret;
		perror("pthread_create");
		status = EXIT_FAILURE;
		goto out;
	}

	pthread_join(sender_thread, NULL);
	pthread_cancel(receiver_thread);
	pthread_join(receiver_thread, NULL);

out:
	sofi_destroy();
	return status;
}
