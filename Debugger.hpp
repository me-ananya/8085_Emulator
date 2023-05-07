#pragma once

#include <iostream>

void debuggerHelp() {
    std::cout << "break or b line no:- It will set break point on given line number." << std::endl;
    std::cout << "run or r :- Run the program until it ends or breakpoint is encountered." << std::endl;
    std::cout << "step or s :- It will run the program one instruction at a time." << std::endl;
    std::cout << "print or p:- It prints the value of register or memory location." << std::endl;
    std::cout << "quit or q:- quits the debugger." << std::endl;
    std::cout << "help:- will show all the commands of debugger." << std::endl;
}