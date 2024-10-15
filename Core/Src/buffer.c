#include <stdint.h>
#include "stdlib.h"
#include "buffer.h"

// Initialize Double Buffer
int32_t *initDbleBuf (dbleBuf_t *buffer, uint32_t size)
{
    buffer->windex = 0;
    buffer->rindex = size;
    buffer->size = size;
    buffer->data = (int32_t *) calloc (2 * size, sizeof(int32_t));
    return buffer->data;
}

// Write to the Double Buffer
void writeDbleBuf (dbleBuf_t *buffer, int32_t entry)
{
    buffer->data[buffer->windex] = entry;
    buffer->windex++;
    if (buffer->windex >= 2 * buffer->size) {
       buffer->windex = 0;
    }
}

// Read Double Buffer
int readDbleBuf (dbleBuf_t *buffer, int32_t *array)
{
    int overrun = (buffer->windex >= buffer->rindex) && !(buffer->windex >= buffer->rindex + buffer->size); // Detect data overrun
    int i;
    for (i = 0; i < buffer->size; i++, (buffer->rindex)++) { // Read the data in one buffer into array
        array[i] = buffer->data[buffer->rindex];
    }
    if (buffer->rindex >= 2*buffer->size) {// Circular buffering
        buffer->rindex = 0;
    }
    return overrun;
}

// Free the Buffer
void freeDbleBuf (dbleBuf_t *buffer)
{
    buffer->windex = 0;
    buffer->rindex = 0;
    buffer->size = 0;
    free (buffer->data);
    buffer->data = NULL;
}
