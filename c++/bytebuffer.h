#pragma once
#include <stddef.h>

struct Buffer {
    size_t length;
    size_t max_length;
    char data[0];
};

typedef struct Buffer Buffer;

Buffer* make_Buffer(const char* data);
/**
 * Take a malloc'd string and realloc it into a Buffer.
 */
Buffer* convert_Buffer(char* data);
Buffer* alloc_Buffer(size_t);
Buffer* realloc_Buffer(Buffer*, size_t);

size_t Buffer_len(Buffer* s);

/*
 * Might modify the first pointer (length extend).
 * Append string b to string a.
 * Also returns the new pointer if u want to use that instead.
 */
Buffer* Buffer_cat(Buffer**, Buffer*);
Buffer* Buffer_cats(Buffer**, const char*);
void Buffer_push(Buffer**, char);
char Buffer_pop(Buffer*);
void Buffer_clear(Buffer*);

/**
 * Big endian buffer.
 */
void Buffer_push_short(Buffer**, int16_t);
void Buffer_push_int(Buffer**, int32_t);
void Buffer_push_double(Buffer**, double);
void Buffer_push_long(Buffer**, int64_t);

/**
 * Same as above but no realloc.
 */
void Buffer_put_short(Buffer*, int16_t);
void Buffer_put_int(Buffer*, int32_t);
void Buffer_put_double(Buffer*, double);
void Buffer_put_long(Buffer*, int64_t);

/**
 * Delete character at index given (0-indexed) and returns it.
 * Does not resize (maxlen) string.
 */
char Buffer_delete(Buffer* s, size_t index);

/**
 * Delete characters [a, b).
 * Does not resize (maxlen) string.
 */
void Buffer_delete_range(Buffer* s, size_t a, size_t b);

/**
 * Postcondition: (*s)->data[index] == c
 */
void Buffer_insert(Buffer** s, size_t index, char c);

#include "bytebuffer.c"
