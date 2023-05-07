#include <iostream>
#include <fstream>
#include <string>
#include "X85.hpp"

int main(int argc, char* argv[]) {
  X85 microProcessor;
  short int in;
  Memory memo;
  microProcessor.reset(memo);
  if(argc>1) {
    if(argv[1][0]=='-' && argv[1][1]=='d') {
      int cycles = 0;
      long nol;
      std::cout << "Enter number of lines of program : ";
      std::cin >> nol;
      char command;
      while(nol--) {
        std::cout << "Address : " << std::hex << microProcessor.getProgramCounter() << std::endl;
        std::cin >> std::hex >>  in;
        memo[microProcessor.getProgramCounter()] = in;
        microProcessor.incrementProgramCounter();
        bool flag = true;
        while(flag) {

          std::cout << " Debugger : " << std::endl;
          std::cout << " Add break : b " << std::endl;
          std::cout << " Run : r " << std::endl;
          std::cout << " Step : s " << std::endl;
          std::cout << " Print : p " << std::endl;
          std::cout << " Quit : q " << std::endl;
          std::cout << " Help : h " << std::endl;
          std::cout << " Skip : k " << std::endl;
          
          std::cin >> command;

          switch(command) {
            case 'b': {
              std::cout << "Add break current : c " << std::endl;
              std::cout << "Add break line number : l " << std::endl;
              std::cin >> command;
              switch(command) {
                case 'c': {
                  memo[microProcessor.getProgramCounter()] = microProcessor.BREAK;
                  microProcessor.incrementProgramCounter();
                }
                case 'l': {
                  std::cout << "Enter Address Byte 1 : ";  
                  std::cin >> std::hex >> in;
                  Word data = in;
                  std::cout << "Enter Address Byte 2 : ";  
                  std::cin >> std::hex >> in;
                  data = (in<<8)|data;
                  memo[data] = microProcessor.BREAK;
                }
              }
              
            } break;
            case 'r': microProcessor.print(memo); break;
            case 's': microProcessor.stepPrint(memo); break;
            case 'p': {
              std::cout << "Print value of Register : r " << std::endl;
              std::cout << "Print value of Address : a " << std::endl;
              std::cin >> command;
              switch(command) {
                case 'r' : microProcessor.printReg(); std::cout << "here"; break;
                case 'a' : {
                    std::cout << "Enter Address Byte 1 : ";  
                    std::cin >> std::hex >> in;
                    Word data = in;
                    std::cout << "Enter Address Byte 2 : ";  
                    std::cin >> std::hex >> in;
                    data = (in<<8)|data;
                    std::cout << data << std::endl;
                    std::cout << std::hex << int(memo[data]) << std::endl;
                }
              }
            } break;
            case 'q': return 0;
            case 'h': break;
            case 'k': flag = false; break;
          }
        }
      }
    }
    else {
      std::ifstream ifs;
      ifs.open(argv[1],std::ifstream::in);
      if(!ifs.is_open()) {
        std::cerr << " Can not open file at : " << argv[1] << std::endl;
        return 0;
      }
      std::string sin;
      while(!ifs.eof()) {
        std::getline(ifs,sin);
        in = std::stoi(sin, 0, 16);
        memo[microProcessor.getProgramCounter()] = in;
        microProcessor.incrementProgramCounter();
      }
      ifs.close();
      microProcessor.print(memo);
    }
  }
  else {
    long nol;
    std::cout << "Enter number of lines of program : ";
    std::cin >> nol;
    while(nol--) {
      std::cout << "Address : " << std::hex << microProcessor.getProgramCounter() << std::endl;
      std::cin >> std::hex >>  in;
      memo[microProcessor.getProgramCounter()] = in;
      microProcessor.incrementProgramCounter();
    }
    microProcessor.print(memo);
  }
  return 0;
}