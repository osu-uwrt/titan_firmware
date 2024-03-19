#ifndef TITAN_TEST_H
#define TITAN_TEST_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define STRINGIFY(A) #A

#define FAIL(reason)                                                                                                   \
    do {                                                                                                               \
        printf("[FAILED ON LINE %d] %s\n", __LINE__, reason);                                                          \
        exit(-1);                                                                                                      \
    } while (0)

#define ASSERT_EQ(expected, actual)                                                                                       \
    do {                                                                                                               \
        if ((expected) != (actual)) {                                                                                  \
            FAIL("Expected " STRINGIFY(actual) " to be " STRINGIFY(expected) " but was not.");                         \
        }                                                                                                              \
    } while (0)

#define ASSERT_TRUE(actual) ASSERT_EQ(true, actual)

#define ASSERT_FALSE(actual) ASSERT_EQ(false, actual)

#endif
