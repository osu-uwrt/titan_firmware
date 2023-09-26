#include <stdio.h>

#include "titan/test.h"
#include "titan/queue.h"

void testQueueEmpty() {
    struct QUEUE_DEFINE(int, 10) queue = { 0 };

    ASSERT_TRUE(QUEUE_EMPTY(&queue));

    QUEUE_CUR_WRITE_ENTRY(&queue);
    QUEUE_MARK_WRITE_DONE(&queue);

    ASSERT_FALSE(QUEUE_EMPTY(&queue));
}

void testQueueFull() {
    struct QUEUE_DEFINE(int, 5) queue = { 0 };


    for(int i = 0; i < 5; i++) {
        ASSERT_FALSE(QUEUE_FULL(&queue));
        QUEUE_CUR_WRITE_ENTRY(&queue);
        QUEUE_MARK_WRITE_DONE(&queue);
    }

    ASSERT_TRUE(QUEUE_FULL(&queue));
}

void testQueueAddRemove() {
    struct QUEUE_DEFINE(int, 5) queue = { 0 };

    for(int i = 0; i < 5; i++) {
        int *write = QUEUE_CUR_WRITE_ENTRY(&queue);
        *write = i;
        QUEUE_MARK_WRITE_DONE(&queue);

        int *read = QUEUE_CUR_READ_ENTRY(&queue);
        ASSERT_EQ(i, *read);
        QUEUE_MARK_READ_DONE(&queue);
    }

    ASSERT_TRUE(QUEUE_EMPTY(&queue));
}

void testQueueAddAllRemoveAll() {
    struct QUEUE_DEFINE(int, 50) queue = { 0 };

    for(int i = 0; i < 50; i++) {
        int *write = QUEUE_CUR_WRITE_ENTRY(&queue);
        *write = i * 5;
        QUEUE_MARK_WRITE_DONE(&queue);
    }

    ASSERT_TRUE(QUEUE_FULL(&queue));

    for(int i = 0; i < 50; i++) {
        int *read = QUEUE_CUR_READ_ENTRY(&queue);
        ASSERT_EQ(i * 5, *read);
        QUEUE_MARK_READ_DONE(&queue);
    }

    ASSERT_TRUE(QUEUE_EMPTY(&queue));
}

int main() {
    testQueueEmpty();
    testQueueFull();
    testQueueAddRemove();
    testQueueAddAllRemoveAll();

    return 0;
}
