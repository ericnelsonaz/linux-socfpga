#include "spij-pkt.h"
#include <unistd.h>

#define SOH_SYMBOL		0x01UL

typedef struct {
	uint8_t	pktHeader;
	uint8_t	streamID;
	uint16_t eventID;
	uint32_t eventData;
	uint32_t timestamp;
} Packet;

int SPIJ_WritePkt (int fd, uint8_t streamID, uint16_t eventID, uint32_t eventData)
{
	Packet packet;
	unsigned len = sizeof(packet)-sizeof(packet.timestamp);

	packet.pktHeader = SOH_SYMBOL;
	packet.streamID = streamID;
	packet.eventID = eventID;
	packet.eventData = eventData;

	return write(fd, &packet, len);
}

int SPIJ_WritePktTS (int fd, uint8_t streamID, uint16_t eventID, uint32_t eventData, uint32_t ts)
{
	Packet packet;
	int bytesXfrd;

	packet.pktHeader = SOH_SYMBOL;
	packet.streamID = streamID;
	packet.eventID = eventID;
	packet.eventData = eventData;
	packet.timestamp = ts;

	return write(fd, &packet, sizeof(packet));
}

#ifdef MODULETEST

#include <fcntl.h>
#include <stdio.h>
#include <sys/time.h>

long long timeval_to_ms (struct timeval const *tv)
{
	return ((long long)tv->tv_sec * 1000)
		+ ((long long)tv->tv_usec / 1000);
}

long long tick_ms (void)
{
	struct timeval now;

	gettimeofday(&now, 0);

	return timeval_to_ms(&now);
}

int main (int argc, char const *argv[])
{
	int err;
	int fd = open(SPIJ_PKT_DEV, O_RDWR);
	if (0 > fd) {
		perror(SPIJ_PKT_DEV);
		return -1;
	}

	err = fcntl(fd, F_SETFD, FD_CLOEXEC);
	if (err) {
		perror("CLOEXEC");
		return -1;
	}
	err = fcntl(fd, F_SETFL, O_NONBLOCK);
	if (err) {
		perror("NONBLOCK");
		return -1;
	}

	unsigned const max = 1;
	long long start;
	unsigned i;

	if (SPIJ_WritePkt(fd, 2, 3, 4)) {
		printf("using 8-byte packets\n");
		start = tick_ms();
		for (i=0; i < max; i++) {
			if (8 != SPIJ_WritePkt (fd, 2, 3, 4)) {
				fprintf(stderr, "error writing packet %d\n", i);
				break;
			}
		}
	} else if (SPIJ_WritePktTS(fd, 2, 3, 4, tick_ms())) {
		printf("using 12-byte packets\n");
		start = tick_ms();
		for (i=0; i < max; i++) {
			if (8 != SPIJ_WritePktTS(fd, 2, 3, 4, tick_ms())) {
				fprintf(stderr, "error writing packet %d\n", i);
				break;
			}
		}
	} else
		printf("neither 8 nor 12-byte packet writes work\n");

	long long end = tick_ms();

	printf("wrote %d packets in %lld ms\n",
	       i, end-start);

	close(fd);
	return 0;
}

#endif
