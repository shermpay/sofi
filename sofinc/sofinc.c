#include <errno.h>
#include <getopt.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include "sofi.h"

static const char *progname = "sofinc";

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

static void usage(bool error)
{
	fprintf(error ? stderr : stdout,
		"Usage: %s [OPTION]...\n"
		"Transmit data over sound, reading from standard input and writing to standard\n"
		"output.\n"
		"\n"
		"Transmission parameters:\n"
		"  -s, --sample-rate=SAMPLE_RATE      set up the streams at SAMPLE_RATE\n"
		"  -f, --frequencies=FREQ0,FREQ1,...  use the given frequencies for symbols,\n"
		"                                     with 2, 4, 16, or 256 frequencies for a\n"
		"                                     symbol width of 1, 2, 4, or 8, respectively\n"
		"  -w, --window=WINDOW_FACTOR         use a window of size WINDOW_FACTOR times\n"
		"                                     the symbol duration time to detect a carrier\n"
		"                                     wave\n"
		"  -g, --gap=GAP_FACTOR               use a gap between packets of size GAP_FACTOR\n"
		"                                     times the symbol duration time\n"
		"  -b, --baud=BAUD                    run at BAUD symbols per second\n"
		"\n"
		"Miscellaneous:\n"
		"  -d                                 increase the debug level by one\n"
		"  --debug-level=DEBUG_LEVEL          set the debug level to DEBUG_LEVEL\n"
		"  -h, --help                         display this help text and exit\n"
		, progname);
	exit(error ? EXIT_FAILURE : EXIT_SUCCESS);
}

int main(int argc, char** argv)
{
	pthread_t sender_thread, receiver_thread;
	int ret;
	int status = EXIT_SUCCESS;
	struct sofi_init_parameters params = DEFAULT_SOFI_INIT_PARAMS;

	if (argc > 0)
		progname = argv[0];
	for (;;) {
		static struct option longopts[] = {
			{"sample-rate",	required_argument,	NULL,	's'},
			{"frequencies",	required_argument,	NULL,	'f'},
			{"window",	required_argument,	NULL,	'w'},
			{"gap",		required_argument,	NULL,	'g'},
			{"baud",	required_argument,	NULL,	'b'},
			{"debug-level",	required_argument,	NULL,	'd'},
			{"help",	no_argument,		NULL,	'h'},
		};
		int opt;
		int longindex;
		char *end;
		float freq;
		int i;

		opt = getopt_long(argc, argv, "b:f:s:w:g:dh",
				  longopts, &longindex);
		if (opt == -1)
			break;

		switch (opt) {
		case 'b':
			params.baud = strtof(optarg, &end);
			if (*end != '\0')
				usage(true);
			if (params.baud < 1.f) {
				fprintf(stderr, "%s: baud must be >=1\n",
					progname);
				usage(true);
			}
			break;
		case 'f':
			for (i = 0; i < 256; i++) {
				if (*optarg == '\0')
					usage(true);
				freq = strtof(optarg, &end);
				if (*end != '\0' && *end != ',')
					usage(true);
				params.symbol_freqs[i] = freq;
				if (*end == '\0')
					break;
				else
					optarg = end + 1;
			}
			if (i == 1) {
				params.symbol_width = 1;
			} else if (i == 3) {
				params.symbol_width = 2;
			} else if (i == 15) {
				params.symbol_width = 4;
			} else if (i == 255) {
				params.symbol_width = 8;
			} else {
				fprintf(stderr, "%s: symbol width must be 1, 2, 4, or 8\n",
					progname);
				usage(true);
			}
			break;
		case 's':
			params.sample_rate = strtol(optarg, &end, 10);
			if (*end != '\0')
				usage(true);
			if (params.sample_rate <= 0) {
				fprintf(stderr, "%s: sample rate must be positive\n",
					progname);
				usage(true);
			}
			break;
		case 'w':
			params.recv_window_factor = strtof(optarg, &end);
			if (*end != '\0')
				usage(true);
			if (params.recv_window_factor <= 0.f) {
				fprintf(stderr, "%s: receiver window factor must be positive\n",
					progname);
				usage(true);
			}
			break;
		case 'g':
			params.interpacket_gap_factor = strtof(optarg, &end);
			if (*end != '\0')
				usage(true);
			if (params.interpacket_gap_factor < 1.f) {
				fprintf(stderr, "%s: interpacket gap factor must be >=1\n",
					progname);
				usage(true);
			}
			break;
		case 'd':
			if (optarg)
				params.debug_level = atoi(optarg);
			else
				params.debug_level++;
			break;
		case 'h':
			usage(false);
		default:
			usage(true);
		}
	}
	ret = sofi_init(&params);
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
