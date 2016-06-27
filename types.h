#include "stdint.h"

typedef struct {
  unsigned long timestamp;
  int dir;
  int omni;
} strength_message_t;

typedef union {
  strength_message_t signalStrength;
  uint8_t buf[];
} msg_buf_t;

typedef struct {
  uint8_t b;
  uint8_t a;
} checksum_t;

typedef union {
  checksum_t checksum;
  uint8_t buf[];
} chk_buf_t;
