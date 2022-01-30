#ifndef _MEMMONITOR_H
#define _MEMMONITOR_H

/**
 * @brief Get the total memory use percentage
 * This includes the stack and statically allocated memory in the total memory calculations
 * 
 * @return int The total memory usage as an integer percentage
 */
int memmonitor_get_total_use_percentage(void);

/**
 * @brief Print stats about memory allocation.
 */
void memmonitor_print_stats(void);

#endif