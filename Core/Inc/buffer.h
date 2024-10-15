#ifndef DOUBLEBUFF_H_
#define DOUBLEBUFF_H_

#include <stdint.h>

typedef struct { // Buffer structure
int32_t size; // Number of entries in  buffer
int32_t windex; // index for writing, mod(2*size)
int32_t rindex; // index for reading, mod(2*size)
int32_t *data; // pointer to the starting address of the double buffer
} dbleBuf_t;

int32_t *initDbleBuf (dbleBuf_t *buffer, uint32_t size);
void writeDbleBuf (dbleBuf_t *buffer, int32_t entry);
int readDbleBuf (dbleBuf_t *buffer, int32_t *array);
void freeDbleBuf (dbleBuf_t *buffer);

#endif /* DOUBLEBUFF_H_ */

