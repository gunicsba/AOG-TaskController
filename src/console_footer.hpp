#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/ioctl.h>
#include <unistd.h>
#endif

class ConsoleFooter {
private:
    std::vector<std::string> footerLines;
    mutable std::mutex footerMutex;
    int consoleWidth;
    int consoleHeight;
    
public:
    ConsoleFooter();
    ~ConsoleFooter();
    
    void updateFooterLine(int lineIndex, const std::string& text);
    void displayFooter();
    void clearFooter();
    void refreshConsole();
    
private:
    void getConsoleSize();
    void setCursorPosition(int x, int y);
    void clearLine();
};

// Global instance for easy access
extern ConsoleFooter g_consoleFooter;