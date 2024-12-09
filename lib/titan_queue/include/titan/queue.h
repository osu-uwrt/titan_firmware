#ifndef QUEUE_H
#define QUEUE_H

#include <stddef.h>

/**
 * @file titan/queue.h
 * @author Robert Pafford
 *
 * FIFO Ring-Buffer Queue Implementation
 *
 * Implementation is designed to have minimal overhead by moving queue operations to macros
 *
 * This implementation is designed to be atomic (sort-of).
 * If one location is writing, it can be interrupted with a read, as well as a read interrupted with a write
 * However, a write cannot be interrupted with another write, nor a read with another read.
 */

/*
 * Guide for using QUEUE macros
 *
 * Example:
 * struct example_struct {
 *     int my_int;
 *     char my_str[8];
 * };
 * static struct QUEUE_DEFINE(struct example_struct, 8) my_queue = {0};
 *
 * void my_put_operation(void) {
 *     // Make sure to check if the queue is full attempting to modify it
 *     if (QUEUE_FULL(&my_queue)) {
 *         return;
 *     }
 *
 *     struct example_struct *entry = QUEUE_CUR_WRITE_ENTRY(&my_queue);
 *     strcpy(entry->my_str, "Example");
 *     entry->my_int = 5;
 *     QUEUE_MARK_WRITE_DONE(&my_queue);
 * }
 *
 * bool my_get_operation(char* data) {
 *     if (QUEUE_EMPTY(&my_queue)) {
 *         return false;
 *     }
 *
 *     struct example_struct *entry = QUEUE_CUR_READ_ENTRY(&my_queue);
 *     printf("Queued: %d\n", entry->my_int);
 *     strcpy(data, entry->my_str);
 *     QUEUE_MARK_READ_DONE(&my_queue);
 *
 *     return true;
 * }
 */

/**
 * Defines a new ring buffer that can fit size elements
 * Note that this allocates size + 1 entries in memory to differentiate between empty and full states
 *
 * This is a struct definition, so use like:
 * static struct QUEUE_DEFINE(int, 8) my_queue;
 *  - or -
 * typedef struct my_queue_struct QUEUE_DEFINE(int, 8) my_queue_t;
 * my_queue_t my_queue1;
 * void do_something(my_queue_t *queue) {...}
 *
 * To create a queue of fixed-size arrays, a typedef will need to be used:
 * typedef char my_string[64];
 * struct QUEUE_DEFINE(my_string, 8) my_queue;
 */
#define QUEUE_DEFINE(type, size)                                                                                       \
    {                                                                                                                  \
        type buf[size + 1];                                                                                            \
        volatile size_t write_pos;                                                                                     \
        volatile size_t read_pos;                                                                                      \
    }

/**
 * (Re)initializes a queue struct object
 *
 * This is not atomic, ensure no other code accesses the array during this operation
 * This can be skipped if the queue is initialized to 0 when defined
 */
#define QUEUE_INIT(queue)                                                                                              \
    do {                                                                                                               \
        (queue)->write_pos = 0;                                                                                        \
        (queue)->read_pos = 0;                                                                                         \
    } while (0)

/**
 * Utility macros
 */
#define QUEUE_RAW_SIZE(queue) (sizeof((queue)->buf) / sizeof(*((queue)->buf)))
#define QUEUE_INDEX_NEXT(queue, index) ((index + 1) % QUEUE_RAW_SIZE(queue))

/**
 * @brief Obtain boolean if queue is empty
 *
 * Examples:
 * if (QUEUE_EMPTY(queue)) {...}
 * bool queue_is_empty = QUEUE_EMPTY(queue);
 */
#define QUEUE_EMPTY(queue) ((queue)->write_pos == (queue)->read_pos)

/**
 * @brief Obtain boolean if queue is full
 *
 * Examples:
 * while (QUEUE_FULL(queue)) {...}
 * bool queue_is_full = QUEUE_FULL(queue);
 */
#define QUEUE_FULL(queue) (QUEUE_INDEX_NEXT(queue, (queue)->write_pos) == (queue)->read_pos)

/**
 * @brief Get a pointer to the next available position to write to.
 * This is _ONLY_ valid when QUEUE_FULL is false
 * The existing data in this entry is undefined. Ensure that all important fields are cleared befure marking done
 *
 * Example:
 * struct QUEUE_DEFINE(int, 8) my_queue;
 * int *current_entry = QUEUE_CUR_WRITE_ENTRY(&my_queue);
 * *current_entry = 5;
 */
#define QUEUE_CUR_WRITE_ENTRY(queue) (&(queue)->buf[(queue)->write_pos])

/**
 * @brief Mark the current write position as ready.
 * Call this after the pointer returned by QUEUE_CUR_WRITE_ENTRY has been successfully written
 *
 * Example:
 * struct QUEUE_DEFINE(int, 8) my_queue;
 * int *current_entry = QUEUE_CUR_WRITE_ENTRY(&my_queue);
 * *current_entry = 5;
 * QUEUE_MARK_WRITE_DONE(&my_queue);
 */
#define QUEUE_MARK_WRITE_DONE(queue)                                                                                   \
    do {                                                                                                               \
        (queue)->write_pos = QUEUE_INDEX_NEXT(queue, (queue)->write_pos);                                              \
    } while (0)

/**
 * @brief Get a pointer for the current entry to be read
 * This is _ONLY_ valid when QUEUE_EMPTY is false
 *
 * Example:
 * struct QUEUE_DEFINE(int, 8) my_queue;
 * int *current_entry = QUEUE_CUR_READ_ENTRY(&my_queue);
 */
#define QUEUE_CUR_READ_ENTRY(queue) (&(queue)->buf[(queue)->read_pos])

/**
 * @brief Mark the current read position as done processing
 * Call this after the pointer returnd by QUEUE_MARK_READ_DONE has been processed and is ready to be re-used for writing
 *
 * Example:
 * struct QUEUE_DEFINE(int, 8) my_queue;
 * int *current_entry = QUEUE_CUR_READ_ENTRY(&my_queue);
 * printf("Queue: %d\n", *current_entry);
 * QUEUE_MARK_READ_DONE(&my_queue);
 */
#define QUEUE_MARK_READ_DONE(queue)                                                                                    \
    do {                                                                                                               \
        (queue)->read_pos = QUEUE_INDEX_NEXT(queue, (queue)->read_pos);                                                \
    } while (0)

#endif
