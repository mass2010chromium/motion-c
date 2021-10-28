#include <stdlib.h>
#include <string.h>

Buffer* make_Buffer(const char* data) {
    Buffer* ret = alloc_Buffer(strlen(data));
    ret->length = strlen(data);
    memcpy(ret->data, data, strlen(data)+1);
    return ret;
}

/**
 * Take a malloc'd string and realloc it into a Buffer.
 */
Buffer* convert_Buffer(char* data) {
    size_t maxlen = strlen(data);
    Buffer* ret = (Buffer*) realloc(data, sizeof(Buffer) + maxlen + 1);
    memmove(ret + 1, ret, maxlen);
    ret->length = maxlen;
    ret->max_length = maxlen;
    return ret;
}

Buffer* alloc_Buffer(size_t maxlen) {
    Buffer* ret = (Buffer*) malloc(sizeof(Buffer) + maxlen + 1);
    ret->data[0] = 0;
    ret->length = 0;
    ret->max_length = maxlen;
    return ret;
}

Buffer* realloc_Buffer(Buffer* s, size_t maxlen) {
    Buffer* ret = (Buffer*) realloc(s, sizeof(Buffer) + maxlen + 1);
    ret->max_length = maxlen;
    if (ret->length > maxlen) {
        ret->length = maxlen;
        ret->data[maxlen] = 0;
    }
    return ret;
}

size_t Buffer_len(Buffer* s) {
    return s->length;
}

/*
 * Might modify the first pointer (length extend).
 * Append string b to string a.
 * Also returns the new pointer if u want to use that instead.
 */
Buffer* Buffer_cat(Buffer** _a, Buffer* b) {
    Buffer* a = *_a;
    Buffer* ret = realloc_Buffer(a, Buffer_len(a) + Buffer_len(b));
    memcpy(ret->data + Buffer_len(ret), b->data, Buffer_len(b)+1);
    ret->length = ret->max_length;
    *_a = ret;
    return ret;
}

void Buffer_push(Buffer** _s, char c) {
    Buffer* s = *_s;
    if (s->length < s->max_length) {
        s->data[s->length] = c;
        s->data[s->length+1] = 0;
        ++s->length;
    }
    else {
        Buffer* _new = realloc_Buffer(s, s->max_length * 2 + 1);
        Buffer_push(&_new, c);
        *_s = _new;
    }
}

char Buffer_pop(Buffer* s) {
    s->length -= 1;
    char ret = s->data[s->length];
    s->data[s->length] = 0;
    return ret;
}

void Buffer_clear(Buffer* s) {
    s->length = 0;
    s->data[0] = 0;
}

/**
 * Delete character at index given (0-indexed) and returns it.
 * Does not resize (maxlen) string.
 */
char Buffer_delete(Buffer* s, size_t index) {
    size_t line_len = s->length;
    size_t rest = line_len - (index + 1);
    char ret = s->data[index];
    s->length -= 1;
    memmove(s->data + index, s->data + index+1, rest+1);
    return ret;
}

/**
 * Delete characters [a, b).
 * Does not resize (maxlen) string.
 */
void Buffer_delete_range(Buffer* s, size_t a, size_t b) {
    size_t line_len = s->length;
    size_t rest = line_len - b;
    s->length -= (b-a);
    memmove(s->data + a, s->data + b, rest+1);
}

/**
 * Postcondition: (*s)->data[index] == c
 */
void Buffer_insert(Buffer** _s, size_t index, char c) {
    Buffer* s = *_s;
    if (s->length == s->max_length) {
        s = realloc_Buffer(s, s->max_length + 1);
        *_s = s;
    }
    size_t tail = s->length - index;
    s->length += 1;
    memmove(s->data+index+1, s->data+index, tail+1);
    s->data[index] = c;
}
