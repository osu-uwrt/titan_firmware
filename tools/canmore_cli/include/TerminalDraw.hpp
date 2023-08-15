#pragma once

#define BUFFER_ALT_ENABLE "\033[?1049h"
#define BUFFER_ALT_DISABLE "\033[?1049l"
#define CURSOR_ENABLE "\033[?25h"
#define CURSOR_DISABLE "\033[?25l"
#define CURSOR_GOTO_START "\033[1;1H"
#define CURSOR_RETURN "\r"
#define CLEAR_TERMINAL "\033[2J"
#define CLEAR_TERMINAL_AFTER "\033[0J"
#define CLEAR_LINE_AFTER "\033[0K"
#define COLOR_RESET "\033[0m"
#define COLOR_TITLE "\033[1;35m"
#define COLOR_HEADER "\033[90m"
#define COLOR_NAME "\033[1;94m"
#define COLOR_NAME_SELECTED "\033[0;34;47m"
#define COLOR_BODY "\033[0;32m"
#define COLOR_NOTICE "\033[0;93m"
#define COLOR_ERROR "\033[1;31m"
#define COLOR_PROMPT "\033[1;32m"

void renderHeader(std::string const &title, int titleWidth = 40);
void renderName(std::string const &name, bool selected = false);
void renderField(std::string const &name, std::string const &value, int nameWidth = 16);
bool keypressAvailable(unsigned int timeoutMs = 0);
