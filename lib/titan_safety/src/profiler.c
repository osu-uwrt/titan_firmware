#include "pico.h"
#include "safety_internal.h"

#include "pico/time.h"

#include <stdbool.h>
#include <stdio.h>

#if SAFETY_ENABLE_PROFILER

// Must be defined in user file
#include "profiler_defs.h"

struct profiler_entry {
    absolute_time_t push_time;
    absolute_time_t pop_time;
    uint64_t total_time;
    uint32_t exception_num;
    uint32_t extra_calls;
    bool has_ran;
    bool is_pushed;

    bool push_printed;
    bool pop_printed;
};

absolute_time_t last_time_reset __attribute__((section(".uninitialized_data.profiler_data")));
struct profiler_entry profiler_data[PROFILER_NUM_DEFS] __attribute__((section(".uninitialized_data.profiler_data")));

void __time_critical_func(profiler_push)(uint32_t profiler_id) {
    // Make sure this isn't called recursively
    assert(!profiler_data[profiler_id].is_pushed);
    assert(profiler_id < PROFILER_NUM_DEFS);

    if (profiler_data[profiler_id].has_ran) {
        profiler_data[profiler_id].extra_calls++;
    }

    profiler_data[profiler_id].exception_num = __get_current_exception();
    profiler_data[profiler_id].push_time = get_absolute_time();
    profiler_data[profiler_id].is_pushed = true;
}

void __time_critical_func(profiler_pop)(uint32_t profiler_id) {
    // Make sure this isn't called before push
    assert(profiler_data[profiler_id].is_pushed);
    assert(profiler_id < PROFILER_NUM_DEFS);

    // Make sure the exception level which pushed this also was the one to pop it
    // If not timelines will get very confusing
    assert(profiler_data[profiler_id].exception_num == __get_current_exception());

    profiler_data[profiler_id].pop_time = get_absolute_time();
    profiler_data[profiler_id].is_pushed = false;
    profiler_data[profiler_id].has_ran = true;
    profiler_data[profiler_id].total_time +=
        absolute_time_diff_us(profiler_data[profiler_id].push_time, profiler_data[profiler_id].pop_time);
}

void __time_critical_func(profiler_reset)(__unused bool check_not_pushed) {
    last_time_reset = get_absolute_time();
    for (int i = 0; i < PROFILER_NUM_DEFS; i++) {
        assert(!check_not_pushed || !profiler_data[i].is_pushed);

        profiler_data[i].has_ran = false;
        profiler_data[i].is_pushed = false;
        profiler_data[i].extra_calls = 0;
        profiler_data[i].total_time = 0;
    }
}

void profiler_dump(void) {
    for (int i = 0; i < PROFILER_NUM_DEFS; i++) {
        profiler_data[i].pop_printed = false;
        profiler_data[i].push_printed = false;
    }

    LOG_INFO("Profiler Log:");

    bool time_found = false;
    int nested_level = 0;
    do {
        time_found = false;
        uint32_t found_profiler_id = 0;
        bool is_push_entry = false;
        absolute_time_t next_time = nil_time;

        for (int i = 0; i < PROFILER_NUM_DEFS; i++) {
            // Check if a time is present and not printed
            if ((profiler_data[i].is_pushed || profiler_data[i].has_ran) && !profiler_data[i].push_printed) {
                // Check if it is the next available time
                if (!time_found || absolute_time_diff_us(next_time, profiler_data[i].push_time) < 0) {
                    time_found = true;
                    next_time = profiler_data[i].push_time;
                    found_profiler_id = i;
                    is_push_entry = true;
                }
            }
            // If push already printed, try checking the pop
            else if (profiler_data[i].has_ran && !profiler_data[i].pop_printed) {
                // Check if it is the next available time
                if (!time_found || absolute_time_diff_us(next_time, profiler_data[i].pop_time) < 0) {
                    time_found = true;
                    next_time = profiler_data[i].pop_time;
                    found_profiler_id = i;
                    is_push_entry = false;
                }
            }
        }

        if (time_found) {
            if (!is_push_entry) {
                nested_level--;
            }

            // Create string space padding
            char nested_spaces[nested_level + 1];
            for (int i = 0; i < nested_level; i++) {
                nested_spaces[i] = ' ';
            }
            nested_spaces[nested_level] = '\0';

            struct profiler_entry *entry = &profiler_data[found_profiler_id];
            if (is_push_entry) {
                entry->push_printed = true;
                int64_t time_us = absolute_time_diff_us(last_time_reset, entry->push_time);
                if (entry->extra_calls > 0) {
                    LOG_INFO(
                        "%sPush %s @%lld us (Exception Level %ld) - %ld Additional Calls occurred (%lld ms total time)",
                        nested_spaces, profiler_names[found_profiler_id], time_us, entry->exception_num,
                        entry->extra_calls, entry->total_time);
                }
                else {
                    LOG_INFO("%sPush %s @%lld us (Exception Level %ld)", nested_spaces,
                             profiler_names[found_profiler_id], time_us, entry->exception_num);
                }
            }
            else {
                entry->pop_printed = true;
                int64_t time_us = absolute_time_diff_us(last_time_reset, entry->pop_time) / 1000;
                int64_t diff_us = absolute_time_diff_us(entry->push_time, entry->pop_time) / 1000;
                LOG_INFO("%sPop %s @%lld us - Time in loop %lld us", nested_spaces, profiler_names[found_profiler_id],
                         time_us, diff_us);
            }

            if (is_push_entry) {
                nested_level++;
            }
        }
    } while (time_found);
}

#endif
