#ifndef LIST_H
#define LIST_H

#include <stddef.h>

/**
 * @file titan/list.h
 * @author Noah Charlton
 *
 * Fixed-capacity unordered list implementation.
 *
 * Notes:
 * - Has a fixed capacity so that the list will always be statically allocated
 * - Remove operations do not perserve the order of the list.
 *
 * Guide: TODO
 */

/**
 * Defines a new queue with the specified type and capacity.
 */
#define LIST_DEFINE(type, size)                                                                                        \
    {                                                                                                                  \
        type buf[size + 1];                                                                                            \
        volatile size_t length;                                                                                        \
    }

/*
 * Returns the amonut of items in the list.
 */
#define LIST_LENGTH(queue) ((queue)->length)

/*
 * Adds an item to the end of the list.
 */
#define LIST_ADD(queue, elem)                                                                                          \
    do {                                                                                                               \
        (queue)->buf[(queue)->length++] = elem;                                                                        \
    } while (0)

#define LIST_GET(queue, idx) ((queue)->buf[idx])

/*
 * Removes an item from the list.
 *
 * If the item removed is the last item the length is decreased.
 * If the item removed is not the last item, the last item will be moved to the index being removed.
 */
#define LIST_REMOVE(queue, idx)                                                                                        \
    do {                                                                                                               \
        if ((queue)->length > idx + 1) {                                                                               \
            (queue)->buf[idx] = (queue)->buf[(queue)->length - 1];                                                     \
        }                                                                                                              \
        (queue)->length--;                                                                                             \
    } while (0)

#endif
