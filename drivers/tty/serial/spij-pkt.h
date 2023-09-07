#ifndef __SPIJ_PKT_H__
#define __SPIJ_PKT_H__

#include <stdint.h>

#define SPIJ_PKT_DEV "/dev/spijpkt"

int SPIJ_WritePkt (int fd, uint8_t streamID, uint16_t eventID, uint32_t eventData);
int SPIJ_WritePktTS (int fd, uint8_t streamID, uint16_t eventID, uint32_t eventData, uint32_t ts);

#endif
