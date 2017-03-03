/*
 * Copyright (c) 2001 Michael B. Allen <mballen@erols.com>
 *
 * The MIT License
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */ 

/* stack.c - a dynamically resizing stack
 */

#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include "stack.h"

#define STACK_INIT_SIZE 32

struct stack {
    unsigned int max_size;
    unsigned int sp;
    unsigned int size;
	unsigned int iter;
    void **array;
};

struct stack *
stack_new(unsigned int max_size)
{
    struct stack *s = malloc(sizeof *s);
    if (s) {
        s->max_size = max_size;
        s->size = max_size < STACK_INIT_SIZE ? max_size : STACK_INIT_SIZE;
        s->sp = 0;
        s->array = malloc(sizeof *s->array * s->size);
        if (s->array == NULL) {
            free(s);
            s = NULL;
        }
    }
    return s;
}
void
stack_del(struct stack *s, void (*free_data_fn)(void *))
{
    if (s == NULL) {
        return;
    }
    if (free_data_fn) {
        while (s->sp > 0) {
            free_data_fn(s->array[--(s->sp)]);
        }
    }
    free(s->array);
    free(s);
}

void
stack_clear(struct stack *s, void (*free_data_fn)(void *))
{
    if (s == NULL) {
        return;
    }
    if (free_data_fn) {
        while (s->sp > 0) {
            free_data_fn(s->array[--(s->sp)]);
        }
    }
}
void
stack_iterate(struct stack *s)
{
	if (s) {
		s->iter = 0;
	}
}
void *
stack_next(struct stack *s)
{
	if (s && s->iter < s->sp) {
		return s->array[s->iter++];
	}
	return NULL;
}
void *
stack_peek(struct stack *s)
{
	if (s == NULL || s->sp == 0) {
		return NULL;
	}
	return s->array[s->sp - 1];
}
int
stack_push(struct stack *s, void *data)
{
    if (s == NULL) {
        return 0;
    }
    if (s->sp == s->size) {
        void **new_array;
        unsigned int new_size;

		if (s->size == s->max_size) {
			return 0;
		}
		if (s->size * 2 > s->max_size) {
			new_size = s->max_size;
		} else {
			new_size = s->size * 2;
		}

        new_array = realloc(s->array, sizeof *s->array * new_size);
        if (new_array == NULL) {
            return 0;
        }
        s->size = new_size;
        s->array = new_array;
    }
    assert(s->sp >= 0 && s->sp <= s->size);
    s->array[s->sp++] = data;
    return 1;
}
void *
stack_pop(struct stack *s)
{
    if (s == NULL || s->sp == 0) {
        return NULL;
    }
    if (s->size >= STACK_INIT_SIZE * 4 && s->sp < s->size / 4) {
        void **new_array;
        unsigned int new_size = s->size / 2;

        new_array = realloc(s->array, sizeof *s->array * new_size);
        if (new_array) {
            s->array = new_array;
            s->size = new_size;
        }
    }
    assert(s->sp > 0 && s->sp <= s->size);
    return s->array[--(s->sp)];
}
int
stack_is_empty(const struct stack *s)
{
    return s == NULL || s->sp == 0;
}
unsigned int
stack_size(const struct stack *s)
{
	return s == NULL ? 0 : s->sp;
}

