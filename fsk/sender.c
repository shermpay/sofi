#include <math.h>
#include <portaudio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER paFramesPerBufferUnspecified

struct callback_data {
	float phase;
};

static int send_callback(const void *input_buffer, void *output_buffer,
			 unsigned long frames_per_buffer,
			 const PaStreamCallbackTimeInfo *time_info,
			 PaStreamCallbackFlags status_flags, void *arg)
{
	float *out = output_buffer;
	struct callback_data *data = arg;

	(void)input_buffer;
	(void)time_info;
	(void)status_flags;

	for (unsigned long i = 0; i < frames_per_buffer; i++) {
		*out++ = sinf(data->phase / 12);
		*out++ = sinf(data->phase / 4);
		data->phase++;
	}

	return paContinue;
}

int main(void)
{
	PaStream *stream;
	PaError err;
	struct callback_data data = {
		.phase = 0.f,
	};
	int status = EXIT_SUCCESS;

	err = Pa_Initialize();
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: initialization failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
		goto out;
	}

	/* Open an audio I/O stream. */
	err = Pa_OpenDefaultStream(&stream, 0, 2, paFloat32, SAMPLE_RATE,
				   FRAMES_PER_BUFFER, send_callback, &data);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: opening stream failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
		goto terminate;
	}

	err = Pa_StartStream(stream);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: starting stream failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
		goto close_stream;
	}

	Pa_Sleep(5000);

	err = Pa_StopStream(stream);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: stopping stream failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
		goto close_stream;
	}

close_stream:
	err = Pa_CloseStream(stream);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: closing stream failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
	}

terminate:
	err = Pa_Terminate();
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: termination failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
	}

out:
	return status;
}
