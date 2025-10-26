#include "console_footer.hpp"
#include <algorithm>
#include <cstring>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/ioctl.h>
#include <unistd.h>
#endif

ConsoleFooter g_consoleFooter;

ConsoleFooter::ConsoleFooter() : footerLines(3) {
    getConsoleSize();
}

ConsoleFooter::~ConsoleFooter() {
    clearFooter();
}

void ConsoleFooter::getConsoleSize() {
#ifdef _WIN32
    CONSOLE_SCREEN_BUFFER_INFO csbi;
    GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi);
    consoleWidth = csbi.srWindow.Right - csbi.srWindow.Left + 1;
    consoleHeight = csbi.srWindow.Bottom - csbi.srWindow.Top + 1;
#else
    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    consoleWidth = w.ws_col;
    consoleHeight = w.ws_row;
#endif
}

void ConsoleFooter::setCursorPosition(int x, int y) {
#ifdef _WIN32
    COORD coord;
    coord.X = x;
    coord.Y = y;
    SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
#else
    printf("\033[%d;%dH", y + 1, x + 1);
#endif
}

void ConsoleFooter::clearLine() {
    // Move to beginning of line and clear to end
#ifdef _WIN32
    CONSOLE_SCREEN_BUFFER_INFO csbi;
    GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi);
    DWORD written;
    FillConsoleOutputCharacter(GetStdHandle(STD_OUTPUT_HANDLE), ' ', consoleWidth, {0, csbi.dwCursorPosition.Y}, &written);
#else
    printf("\033[2K"); // Clear entire line
#endif
}

void ConsoleFooter::updateFooterLine(int lineIndex, const std::string& text) {
    std::lock_guard<std::mutex> lock(footerMutex);
    if (lineIndex >= 0 && lineIndex < static_cast<int>(footerLines.size())) {
        footerLines[lineIndex] = text;
    }
}

void ConsoleFooter::displayFooter() {
    std::lock_guard<std::mutex> lock(footerMutex);
    getConsoleSize();
    
    // Position at the footer area (last few lines)
    int footerStartLine = std::max(0, consoleHeight - static_cast<int>(footerLines.size()) - 1);
    
    for (size_t i = 0; i < footerLines.size(); ++i) {
        setCursorPosition(0, footerStartLine + static_cast<int>(i));
        clearLine();
        
        // Truncate or pad the text to fit console width
        std::string displayText = footerLines[i];
        if (static_cast<int>(displayText.length()) > consoleWidth) {
            displayText = displayText.substr(0, consoleWidth - 3) + "...";
        }
        
        std::cout << displayText;
    }
    
    // Move cursor back to a reasonable position for normal output
    setCursorPosition(0, footerStartLine - 1);
    std::cout.flush();
}

void ConsoleFooter::clearFooter() {
    std::lock_guard<std::mutex> lock(footerMutex);
    getConsoleSize();
    
    int footerStartLine = std::max(0, consoleHeight - static_cast<int>(footerLines.size()) - 1);
    
    for (size_t i = 0; i < footerLines.size(); ++i) {
        setCursorPosition(0, footerStartLine + static_cast<int>(i));
        clearLine();
    }
}

void ConsoleFooter::refreshConsole() {
    // This would be called periodically or when needed
    displayFooter();
}