#include "console_footer.hpp"
#include <iostream>
#include <thread>
#include <chrono>

// Example usage of the console footer
void updateFooterExample() {
    // Update footer lines with relevant information
    g_consoleFooter.updateFooterLine(0, "Status: Running | Connected Clients: 3 | Memory: 45MB");
    g_consoleFooter.updateFooterLine(1, "Last Message: Tramline sequence updated | Sequence #: 12");
    g_consoleFooter.updateFooterLine(2, "Press Ctrl+C to exit | FPS: 60");
    
    // Display the footer
    g_consoleFooter.displayFooter();
}

int main() {
    std::cout << "Console Application with Footer Bar" << std::endl;
    std::cout << "===================================" << std::endl;
    std::cout << "This demo shows a footer bar at the bottom of the console." << std::endl;
    std::cout << std::endl;
    
    // Initialize and display the footer
    updateFooterExample();
    
    // Simulate some application activity
    for (int i = 0; i < 10; ++i) {
        std::cout << "Application log message #" << (i + 1) << std::endl;
        
        // Update footer with changing information
        g_consoleFooter.updateFooterLine(0, "Status: Running | Connected Clients: 3 | Memory: " + std::to_string(45 + i) + "MB");
        g_consoleFooter.updateFooterLine(1, "Last Message: Processing data | Item: " + std::to_string(i + 1));
        g_consoleFooter.updateFooterLine(2, "Press Ctrl+C to exit | Time: " + std::to_string(i) + "s");
        g_consoleFooter.displayFooter();
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    std::cout << "Demo completed. Press Enter to exit." << std::endl;
    std::cin.get();
    
    return 0;
}