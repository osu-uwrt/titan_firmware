#include <stdio.h>

#include "titan/test.h"
#include "titan/list.h"

void testListAdd() {
    struct LIST_DEFINE(int, 10) list = { 0 };

    ASSERT_EQ(0, LIST_LENGTH(&list));
    LIST_ADD(&list, 15);
    ASSERT_EQ(1, LIST_LENGTH(&list));
    LIST_ADD(&list, 50);
    ASSERT_EQ(2, LIST_LENGTH(&list));

    ASSERT_EQ(15, LIST_GET(&list, 0));
    ASSERT_EQ(50, LIST_GET(&list, 1));
}

void testListRemoveFront() {
    struct LIST_DEFINE(int, 10) list = { 0 };

    LIST_ADD(&list, 123);
    LIST_ADD(&list, 456);
    LIST_ADD(&list, 789);

    LIST_REMOVE(&list, 0);

    ASSERT_EQ(2, LIST_LENGTH(&list));
    ASSERT_EQ(789, LIST_GET(&list, 0));
    ASSERT_EQ(456, LIST_GET(&list, 1));
}

void testListRemoveLast() {
    struct LIST_DEFINE(int, 10) list = { 0 };

    LIST_ADD(&list, 1000);
    LIST_ADD(&list, 2000);
    LIST_ADD(&list, 3000);

    LIST_REMOVE(&list, 2);

    ASSERT_EQ(2, LIST_LENGTH(&list));
    ASSERT_EQ(1000, LIST_GET(&list, 0));
    ASSERT_EQ(2000, LIST_GET(&list, 1));
}

int main() {
    testListAdd();
    testListRemoveFront();
    testListRemoveLast();
}
