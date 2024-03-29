#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "sofi.h"

#define MAX_MESSAGE_LENGTH (sizeof(((struct sofi_packet *)0)->payload))

static const char *progname = "sofinc";
static bool keep_open;
static size_t max_message_length = MAX_MESSAGE_LENGTH;

static pthread_t sender_thread, receiver_thread;

static void *sender_loop(void *receiver)
{
	struct sofi_packet packet;
	void *status = (void *)0;
	int ret;

	for (;;) {
		packet.len = fread(packet.payload, 1, max_message_length,
				   stdin);
		if (packet.len == 0)
			break;
		sofi_send(&packet);
	}
	packet.len = 0;
	sofi_send(&packet);
	if (ferror(stdin) && errno != EINTR) {
		perror("fread");
		status = (void *)-1;
	}
	if (receiver) {
		ret = pthread_cancel(receiver_thread);
		assert(ret == 0);
	}
	return status;
}

static void *receiver_loop(void *sender)
{
	struct sofi_packet packet;
	void *status = (void *)0;
	int ret;

	for (;;) {
		sofi_recv(&packet);
		if (packet.len == 0 && !keep_open) {
			if (fclose(stdout))
				perror("fclose");
			break;
		}
		fwrite(packet.payload, 1, packet.len, stdout);
		fflush(stdout);
		if (ferror(stdout)) {
			perror("fflush");
			status = (void *)-1;
			break;
		}
	}
	if (sender) {
		ret = pthread_cancel(sender_thread);
		assert(ret == 0);
	}
	return status;
}

static void usage(bool error)
{
	fprintf(error ? stderr : stdout,
		"Usage: %s [OPTION]...\n"
		"Transmit data over sound, reading from standard input and writing to standard\n"
		"output.\n"
		"\n"
		"Communication direction:\n"
		"  -R, --receiver                     run the receiver (enabled by default unless\n"
		"                                     --sender is given)\n"
		"  -S, --sender                       run the sender (enabled by default unless\n"
		"                                     --receiver is given)\n"
		"Transmission parameters:\n"
		"  -b, --baud=BAUD                    run at BAUD symbols per second\n"
		"  -f, --frequencies=FREQ0,FREQ1,...  use the given frequencies for symbols,\n"
		"                                     with 2, 4, 16, or 256 frequencies for a\n"
		"                                     symbol width of 1, 2, 4, or 8, respectively\n"
		"  -g, --gap=GAP_FACTOR               use a gap between packets of size GAP_FACTOR\n"
		"                                     times the symbol duration time\n"
		"  -l, --max-length=LENGTH            send packets of at most LENGTH bytes\n"
		"  -s, --sample-rate=SAMPLE_RATE      set up the streams at SAMPLE_RATE\n"
		"  -w, --window=WINDOW_FACTOR         use a window of size WINDOW_FACTOR times\n"
		"                                     the symbol duration time to detect a carrier\n"
		"                                     wave\n"
		"\n"
		"Miscellaneous:\n"
		"  -k, --keep-open                    keep the connection open even if the sender\n"
		"                                     closes it\n"
		"  -d                                 increase the debug level by one\n"
		"  --debug-level=DEBUG_LEVEL          set the debug level to DEBUG_LEVEL\n"
		"  -h, --help                         display this help text and exit\n"
		, progname);
	exit(error ? EXIT_FAILURE : EXIT_SUCCESS);
}

int main(int argc, char** argv)
{
	int ret;
	int status = EXIT_SUCCESS;
	void *retval;
	struct sofi_init_parameters params = DEFAULT_SOFI_INIT_PARAMS;
	params.sender = false;
	params.receiver = false;

	if (argc > 0)
		progname = argv[0];
	for (;;) {
		static struct option longopts[] = {
			{"receiver",	no_argument,		NULL,	'R'},
			{"sender",	no_argument,		NULL,	'S'},
			{"baud",	required_argument,	NULL,	'b'},
			{"frequencies",	required_argument,	NULL,	'f'},
			{"gap",		required_argument,	NULL,	'g'},
			{"max-length",	required_argument,	NULL,	'l'},
			{"sample-rate",	required_argument,	NULL,	's'},
			{"window",	required_argument,	NULL,	'w'},
			{"keep-open",	no_argument,		NULL,	'k'},
			{"debug-level",	required_argument,	NULL,	'd'},
			{"help",	no_argument,		NULL,	'h'},
		};
		int opt;
		int longindex;
		char *end;
		float freq;
		int i;

		opt = getopt_long(argc, argv, "RSb:f:g:l:s:w:kdh",
				  longopts, &longindex);
		if (opt == -1)
			break;

		switch (opt) {
		case 'R':
			params.receiver = true;
			break;
		case 'S':
			params.sender = true;
			break;
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
		case 'l':
			max_message_length = (size_t)strtoul(optarg, &end, 10);
			if (*end != '\0')
				usage(true);
			if (max_message_length == 0 ||
			    max_message_length > MAX_MESSAGE_LENGTH) {
				fprintf(stderr, "%s: max message length must be non-zero and <=%lu\n",
					progname, (unsigned long)MAX_MESSAGE_LENGTH);
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
		case 'k':
			keep_open = true;
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
	if (!params.sender && !params.receiver)
		params.sender = params.receiver = true;

	ret = sofi_init(&params);
	if (ret)
		return EXIT_FAILURE;

	if (params.sender) {
		ret = pthread_create(&sender_thread, NULL, sender_loop,
				     (void *)params.receiver);
		if (ret) {
			errno = ret;
			perror("pthread_create");
			status = EXIT_FAILURE;
			goto out;
		}
	}
	if (params.receiver) {
		ret = pthread_create(&receiver_thread, NULL, receiver_loop,
				     (void *)params.sender);
		if (ret) {
			if (params.sender) {
				pthread_cancel(sender_thread);
				pthread_join(sender_thread, NULL);
			}
			errno = ret;
			perror("pthread_create");
			status = EXIT_FAILURE;
			goto out;
		}
	}

	if (params.sender) {
		ret = pthread_join(sender_thread, &retval);
		assert(ret == 0);
		if (retval)
			status = EXIT_FAILURE;
	}
	if (params.receiver) {
		ret = pthread_join(receiver_thread, &retval);
		assert(ret == 0);
		if (retval)
			status = EXIT_FAILURE;
	}

out:
	sofi_destroy();
	return status;
}
