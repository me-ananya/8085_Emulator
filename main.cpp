#include <iostream>
#include <array>
#include <fstream>
#include <string>
#define Byte unsigned char
#define Word unsigned short

struct Memory {
  static constexpr unsigned MEM_LIMIT = 1024 * 64;
  std::array<Byte, MEM_LIMIT> data;

  void initialise();

  // Read a byte
  Byte operator[](unsigned p_address) const { return data[p_address]; }

  // Write a byte
  Byte &operator[](unsigned p_address) { return data[p_address]; }
};

struct StatusFlags {
  bool CY, P, AC, Z, S;
};

class X85 {
public:
  X85() {}
  X85(const X85 &other);
  X85(X85 &&other) noexcept = delete;
  virtual ~X85() noexcept {}

  void reset(Memory &mem);
  void execute(int &cycles, Memory &mem);

  Word readRegisterPair(const Byte &reg1, const Byte &reg2);
  void writeRegisterPair(Byte &reg1, Byte &reg2, Word &data);
  Byte fetchByte(int &cycles, const Memory &mem);
  Word fetchWord(int &cycles, const Memory &mem);

  /** opcodes */

  static constexpr Byte
      /*  Data Transfer group */

      // Move
      MOV_AB = 0x78, MOV_AC = 0x79, MOV_AD = 0x7A, MOV_AE = 0x7B, MOV_AH = 0x7C,
      MOV_AL = 0x7D, MOV_AM = 0x7E,

      MOV_BA = 0x47, MOV_BC = 0x41, MOV_BD = 0x42, MOV_BE = 0x43,
      MOV_BH = 0x44, MOV_BL = 0x45, MOV_BM = 0x46,

      MOV_CA = 0x4F, MOV_CB = 0x48, MOV_CD = 0x4A, MOV_CE = 0x4B,
      MOV_CH = 0x4C, MOV_CL = 0x4D, MOV_CM = 0x4E,

      MOV_DA = 0x57, MOV_DB = 0x50, MOV_DC = 0x51, MOV_DE = 0x53,
      MOV_DH = 0x54, MOV_DL = 0x55, MOV_DM = 0x56,

      MOV_EA = 0x5F, MOV_EB = 0x58, MOV_EC = 0x59, MOV_ED = 0x5A,
      MOV_EH = 0x5C, MOV_EL = 0x5D, MOV_EM = 0x5E,

      MOV_HA = 0x67, MOV_HB = 0x60, MOV_HC = 0x61, MOV_HD = 0x62, MOV_HE = 0x63,
      MOV_HL = 0x65, MOV_HM = 0x66,

      MOV_LA = 0x6F, MOV_LB = 0x68, MOV_LC = 0x69, MOV_LD = 0x6A, MOV_LE = 0x6B,
      MOV_LH = 0x6C, MOV_LM = 0x6E,

      MOV_MA = 0x77, MOV_MB = 0x70, MOV_MC = 0x71, MOV_MD = 0x72, MOV_ME = 0x73,
      MOV_MH = 0x74, MOV_ML = 0x75,

      // Move immediate
      MVI_A = 0x3E, MVI_B = 0x06, MVI_C = 0x0E, MVI_D = 0x16, MVI_E = 0x1E,
      MVI_H = 0x26, MVI_L = 0x2E, MVI_M = 0x36,

      // load immediate (reg pair)
      LXI_B = 0x01, LXI_D = 0x11, LXI_H = 0x21, LXI_SP = 0x31,

      // load/store A direct ( from register pair )
      LDAX_B = 0x0A, LDAX_D = 0x1A, STAX_B = 0x02, STAX_D = 0x12,

      // load/store A direct ( from memory )
      LDA = 0x3A, STA = 0x32,

      // load/store HL direct
      LHLD = 0x2A, SHLD = 0x22,

      // exchange HL and DE
      XCHNG = 0xEB,

      /** Branch group*/

      // Jump
      JMP = 0xC3, JNZ = 0xC2, JZ = 0xCA, JNC = 0xD2, JC = 0xDA, JPO = 0xE2,
      JPE = 0xEA, JP = 0xF2, JM = 0xFA,

      // Call
      CALL = 0xCD, CNZ = 0xC4, CZ = 0xCC, CNC = 0xD4, CC = 0xDC, CPO = 0xE6,
      CPE = 0xEC, CP = 0xF4, CM = 0xFC,

      // Return
      RET = 0xC9, RNZ = 0xC0, RZ = 0xC8, RNC = 0xD0, RC = 0xD8, RPO = 0xE0,
      RPE = 0xE8, RP = 0xF0, RM = 0xF8,

      // jump indirect
      PCHL = 0xE9,

      //halt
      HLT = 0x76,
      //set
      SET = 0x03;

  /** getters */
  auto getProgramCounter() const { return PC; }
  auto getStackPointer() const { return SP; }
  auto getStatusFlags() const { return statusFlags; }

  auto getARegister() const { return A; }
  auto getBRegister() const { return B; }
  auto getCRegister() const { return C; }
  auto getDRegister() const { return D; }
  auto getERegister() const { return E; }
  auto getHRegister() const { return H; }
  auto getLRegister() const { return L; }

  /* setters */
  auto incrementProgramCounter() {PC++;}
  auto resetProgramCounter() {PC = 0;}

  /*Print*/
  void print(Memory &mem);


private:
  StatusFlags statusFlags;
  Word PC;
  Word SP;

  Byte A, B, C, D, E, H, L;

  /** instruction set */
  void setMemory(int &cycles, Memory &mem);
  void loadAccumulator(int &cycles, const Memory &mem);
  void storeAccumulator(int &cycles, Memory &mem);

  void moveRegister(Byte &destination, Byte &source);
  void moveIndirectTo(Byte &destination, const Memory &mem, int &cycles);
  void moveIndirectFrom(const Byte &source, Memory &mem, int &cycles);
  void moveImmediate(Byte &destination, int &cycles, const Memory &mem);
  void loadImmediate(Byte &reg1, Byte &reg2, int &cycles, const Memory &mem);
  void loadImmediate(Word &reg, int &cycles, const Memory &mem);

  void unconditionalJump(int &cycles, const Memory &mem);

  // conditional jump
  void jumpIfNotZero(int &cycles, const Memory &mem);
  void jumpIfZero(int &cycles, const Memory &mem);
  void jumpIfNotCarry(int &cycles, const Memory &mem);
  void jumpIfCarry(int &cycles, const Memory &mem);
  void jumpIfOddParity(int &cycles, const Memory &mem);
  void jumpIfEvenParity(int &cycles, const Memory &mem);
  void jumpIfPositive(int &cycles, const Memory &mem);
  void jumpIfMinus(int &cycles, const Memory &mem);

  // call
  void unconditionalCall(int &cycles, Memory &mem);

  // conditional call
  void callIfNotZero(int &cycles, Memory &mem);
  void callIfZero(int &cycles, Memory &mem);
  void callIfNotCarry(int &cycles, Memory &mem);
  void callIfCarry(int &cycles, Memory &mem);
  void callIfOddParity(int &cycles, Memory &mem);
  void callIfEvenParity(int &cycles, Memory &mem);
  void callIfPositive(int &cycles, Memory &mem);
  void callIfMinus(int &cycles, Memory &mem);

  // stack
  void stackPush(int &cycles, Memory &mem, const Byte &data);
  Byte stackPop(int &cycles, const Memory &mem);
};



void Memory::initialise() { data = {}; }

void X85::reset(Memory &mem) {
  PC = A = B = C = D = E = H = L = 0;
  SP = 0xFFFF;
  statusFlags = {};
  mem.initialise();
}

void X85::execute(int &cycles, Memory &mem) {
  while (true) {
    auto lastInstructionLocation = PC;
    auto instruction = fetchByte(cycles, mem);

    switch (instruction) {
    case HLT: return;
    case SET: setMemory(cycles,mem);break;

    case MOV_AB: moveRegister(A, B);break;
    case MOV_AC: moveRegister(A, C);break;
    case MOV_AD: moveRegister(A, D);break;
    case MOV_AE: moveRegister(A, E);break;
    case MOV_AH: moveRegister(A, H);break;
    case MOV_AL: moveRegister(A, L);break;

    case MOV_BA: moveRegister(B, A);break;
    case MOV_BC: moveRegister(B, C);break;
    case MOV_BD: moveRegister(B, D);break;
    case MOV_BE: moveRegister(B, E);break;
    case MOV_BH: moveRegister(B, H);break;
    case MOV_BL: moveRegister(B, L);break;

    case MOV_CA: moveRegister(C, A);break;
    case MOV_CB: moveRegister(C, B);break;
    case MOV_CD: moveRegister(C, D);break;
    case MOV_CE: moveRegister(C, E);break;
    case MOV_CH: moveRegister(C, H);break;
    case MOV_CL: moveRegister(C, L);break;

    case MOV_DA: moveRegister(D, A);break;
    case MOV_DB: moveRegister(D, B);break;
    case MOV_DC: moveRegister(D, C);break;
    case MOV_DE: moveRegister(D, E);break;
    case MOV_DH: moveRegister(D, H);break;
    case MOV_DL: moveRegister(D, L);break;

    case MOV_EA: moveRegister(E, A);break;
    case MOV_EB: moveRegister(E, B);break;
    case MOV_EC: moveRegister(E, C);break;
    case MOV_ED: moveRegister(E, E);break;
    case MOV_EH: moveRegister(E, H);break;
    case MOV_EL: moveRegister(E, L);break;

    case MOV_HA: moveRegister(H, A);break;
    case MOV_HB: moveRegister(H, B);break;
    case MOV_HC: moveRegister(H, C);break;
    case MOV_HD: moveRegister(H, E);break;
    case MOV_HE: moveRegister(H, E);break;
    case MOV_HL: moveRegister(H, L);break;

    case MOV_LA: moveRegister(L, A);break;
    case MOV_LB: moveRegister(L, B);break;
    case MOV_LC: moveRegister(L, C);break;
    case MOV_LD: moveRegister(L, E);break;
    case MOV_LH: moveRegister(L, H);break;
    case MOV_LE: moveRegister(L, E);break;

    // register indirect addressing mode
    case MOV_AM: moveIndirectTo(A, mem, cycles);break;
    case MOV_BM: moveIndirectTo(B, mem, cycles);break;
    case MOV_CM: moveIndirectTo(A, mem, cycles);break;
    case MOV_DM: moveIndirectTo(A, mem, cycles);break;
    case MOV_EM: moveIndirectTo(E, mem, cycles);break;
    case MOV_HM: moveIndirectTo(H, mem, cycles);break;
    case MOV_LM: moveIndirectTo(L, mem, cycles);break;

    case MOV_MA: moveIndirectFrom(A, mem, cycles);break;
    case MOV_MB: moveIndirectFrom(B, mem, cycles);break;
    case MOV_MC: moveIndirectFrom(A, mem, cycles);break;
    case MOV_MD: moveIndirectFrom(A, mem, cycles);break;
    case MOV_ME: moveIndirectFrom(E, mem, cycles);break;
    case MOV_MH: moveIndirectFrom(H, mem, cycles);break;
    case MOV_ML: moveIndirectFrom(L, mem, cycles);break;

    // move immediate
    case MVI_A: moveImmediate(A, cycles, mem);break;
    case MVI_B: moveImmediate(B, cycles, mem);break;
    case MVI_C: moveImmediate(C, cycles, mem);break;
    case MVI_D: moveImmediate(D, cycles, mem);break;
    case MVI_E: moveImmediate(E, cycles, mem);break;
    case MVI_H: moveImmediate(H, cycles, mem);break;
    case MVI_L: moveImmediate(L, cycles, mem);break;
    case MVI_M: {
      // memory address to where the data should be tranfered would be
      // present in H-L register pair
      auto address = readRegisterPair(H, L);
      cycles++;
      auto destination = mem[address];
      moveImmediate(destination, cycles, mem);
    } break;

    // load/store immediate (register pair)
    case LXI_B: loadImmediate(B, C, cycles, mem);break;
    case LXI_D: loadImmediate(D, E, cycles, mem);break;
    case LXI_H: loadImmediate(H, L, cycles, mem);break;
    case LXI_SP: loadImmediate(SP, cycles, mem);break;

    // load/store accumulator ( from register pair )
    case LDAX_B: {
      auto address = readRegisterPair(B, C);
      cycles++;
      A = mem[address];
    } break;
    case LDAX_D: {
      auto address = readRegisterPair(D, E);
      cycles++;
      A = mem[address];
    } break;
    case STAX_B: {
      auto address = readRegisterPair(B, C);
      cycles++;
      mem[address] = A;
    } break;
    case STAX_D: {
      auto address = readRegisterPair(D, E);
      cycles++;
      mem[address] = A;
    } break;

    // load store accumulator directly
    case LDA: loadAccumulator(cycles, mem); break;
    case STA: storeAccumulator(cycles, mem); break;
    // load/store HL direct
    case LHLD: {
      auto address = fetchWord(cycles, mem);
      H = mem[address];
      L = mem[address + 1];
    } break;
    case SHLD: {
      auto address = fetchWord(cycles, mem);
      mem[address] = L;
      mem[address + 1] = H;
    } break;

    // exchange HL and DE
    case XCHNG: {
      auto dataHL = readRegisterPair(H, L);
      auto dataDE = readRegisterPair(D, E);
      writeRegisterPair(H, L, dataDE);
      writeRegisterPair(D, E, dataHL);
    } break;

    // jump
    case JMP: unconditionalJump(cycles, mem);break;

    // contitional jumps
    case JNZ: jumpIfNotZero(cycles, mem);break;
    case JZ: jumpIfZero(cycles, mem);break;
    case JNC: jumpIfNotCarry(cycles, mem);break;
    case JC: jumpIfCarry(cycles, mem);break;
    case JPO: jumpIfOddParity(cycles, mem);break;
    case JP: jumpIfEvenParity(cycles, mem);break;
    case JPE: jumpIfPositive(cycles, mem);break;
    case JM: jumpIfMinus(cycles, mem);break;

    // call instructions
    case CALL: unconditionalCall(cycles, mem);break;

    // conditional call
    case CNZ: callIfNotZero(cycles, mem);break;
    case CZ: callIfZero(cycles, mem);break;
    case CNC: callIfNotCarry(cycles, mem);break;
    case CC: callIfCarry(cycles, mem);break;
    case CPO: callIfOddParity(cycles, mem);break;
    case CP: callIfEvenParity(cycles, mem);break;
    case CPE: callIfPositive(cycles, mem);break;
    case CM: callIfMinus(cycles, mem);break;

    default:
      // throw for unknown instruction.
      std::cerr << "unknown instruction enountered "
                << (int)instruction
                << " at memory location : " << lastInstructionLocation << "\n";
                return;
    }
  }
}

void X85::setMemory(int &cycles, Memory &mem) {
  auto address = fetchWord(cycles, mem);
  auto value = fetchByte(cycles, mem);
  mem[address] = value;
}

/**
 * Read the 16-bit (2-Bytes/1-Word) data from register pair reg1-reg2.
 */
Word X85::readRegisterPair(const Byte &reg1, const Byte &reg2) {
  // TODO: assert that reg1 and reg2 are valid register pair.
  Word data = reg1 << 8;
  data |= reg2;
  return data;
}

/**
 * Write 16-bit (2-Bytes/1-Word) data to register pair reg1-reg2.
 */
void X85::writeRegisterPair(Byte &reg1, Byte &reg2, Word &data) {
  reg1 = data >> 8;
  reg2 = (Byte)data;
}

/**
 * Fetch one byte data pointed by the Progam counter from memory MEM.
 */
Byte X85::fetchByte(int &cycles, const Memory &mem) {
  auto currentData = mem[PC];
  cycles++;
  PC++;
  return currentData;
}

/**
 * Fetch one word ( 2-bytes ) data pointed by the Progam counter from
 * memory MEM. Assuming the platform is little-endien, we would first
 * find the lower byte of address then lower byte.
 */
Word X85::fetchWord(int &cycles, const Memory &mem) {
  Word currentData = mem[PC];
  PC++;
  currentData |= (mem[PC] << 8);
  PC++;
  cycles += 2;
  return currentData;
}

/** Instruction set */

/**
 * Load the content of accumulator (A register). The contents of a
 * memory location, specified by a 16-bit address in the operand, are
 * copied to the accumulator.
 */
void X85::loadAccumulator(int &cycles, const Memory &mem) {
  auto address = fetchWord(cycles, mem);
  A = mem[address];
}

/**
 * The contents of the accumulator are copied into the memory location
 * specified by the operand.
 * This is a 3-byte instruction, the second byte specifies the low-order
 * address and the third byte specifies the high-order address.
 */
void X85::storeAccumulator(int &cycles, Memory &mem) {
  Word address = fetchWord(cycles, mem);
  mem[address] = A;
}


void X85::moveRegister(Byte &destination, Byte &source) {
    destination = source;
}

void X85::moveIndirectTo(Byte &destination, const Memory &mem, int &cycles) {
  auto address = readRegisterPair(H, L);
  cycles++;
  destination = mem[address];
}

/**
 * Move the contents of SOURCE register to the memory location pointed
 * by the H-L pair.
 */
void X85::moveIndirectFrom(const Byte &source, Memory &mem, int &cycles) {
  auto address = readRegisterPair(H, L);
  cycles++;
  mem[address] = source;
}

/**
 * Move the 8-bit(1-byte) data (provided as operand with the
 * instruction) in the DESTINATION register or memory.
 */
void X85::moveImmediate(Byte &destination, int &cycles, const Memory &mem) {
  auto data = fetchByte(cycles, mem);
  destination = data;
}

/**
 * Load the 16-bit address into 8-bit register pair (reg1,reg2)
 */
void X85::loadImmediate(Byte &reg1, Byte &reg2, int &cycles,
                        const Memory &mem) {
  auto data = fetchWord(cycles, mem);
  writeRegisterPair(reg1, reg2, data);
}

/**
 * Load the 16-bit address into the 16 bit register reg (currently
 * only stack pointer)
 */
void X85::loadImmediate(Word &reg, int &cycles, const Memory &mem) {
  auto data = fetchWord(cycles, mem);
  reg = data;
}

/**
 * Transfer program control to a certain memory location
 * unconditionally.
 */
void X85::unconditionalJump(int &cycles, const Memory &mem) {
  auto destination = fetchWord(cycles, mem);
  PC = mem[destination];
}

/**
 * Transfer program control to a certain memory location only when
 * zero flag is not set.
 */
void X85::jumpIfNotZero(int &cycles, const Memory &mem) {
  if (!statusFlags.Z) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the zero flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to jumpIfNotZero Transfer program control to a certain
 * memory location only when zero flag is set.
 */
void X85::jumpIfZero(int &cycles, const Memory &mem) {
  if (statusFlags.Z) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the zero flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Transfer program control to a certain memory location only when
 * carry flag is not set.
 */
void X85::jumpIfNotCarry(int &cycles, const Memory &mem) {
  if (!statusFlags.CY) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the carry flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to jumpIfNotCarry Transfer program control to a certain
 * memory location only when carry flag is set.
 */
void X85::jumpIfCarry(int &cycles, const Memory &mem) {
  if (statusFlags.CY) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the carry flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Transfer program control to a certain memory location only when
 * carry flag is not set.
 */
void X85::jumpIfOddParity(int &cycles, const Memory &mem) {
  if (!statusFlags.P) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the parity flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to jumpIfOddParity. Transfer program control to a certain
 * memory location only when parity flag is set.
 */
void X85::jumpIfEvenParity(int &cycles, const Memory &mem) {
  if (statusFlags.P) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the parity flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Transfer program control to a certain memory location only when
 * sign flag is not set.
 */
void X85::jumpIfPositive(int &cycles, const Memory &mem) {
  if (!statusFlags.S) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the sign flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to jumpIfPositive Transfer program control to a certain
 * memory location only when sign flag is set.
 */
void X85::jumpIfMinus(int &cycles, const Memory &mem) {
  if (statusFlags.S) {
    unconditionalJump(cycles, mem);
  } else {
    // In case the sign flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Read the Target address which would be provided to the instruction
 * as operand, and then branch out to the subroutine present at that
 * address. Similar to jump instruction but also stores the returns
 * address on the stack so that the flow can return when the next
 * return command is encountered.
 */
void X85::unconditionalCall(int &cycles, Memory &mem) {
  auto destination = fetchWord(cycles, mem);
  auto returnAddress = mem[PC];
  stackPush(cycles, mem, returnAddress);
  PC = mem[destination];
}

/**
 * Transfer program control to a certain memory location only when
 * zero flag is not set.
 */
void X85::callIfNotZero(int &cycles, Memory &mem) {
  if (!statusFlags.Z) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the zero flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to callIfNotZero Transfer program control to a certain
 * memory location only when zero flag is set.
 */
void X85::callIfZero(int &cycles, Memory &mem) {
  if (statusFlags.Z) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the zero flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Transfer program control to a certain memory location only when
 * carry flag is not set.
 */
void X85::callIfNotCarry(int &cycles, Memory &mem) {
  if (!statusFlags.CY) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the carry flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to callIfNotCarry Transfer program control to a certain
 * memory location only when carry flag is set.
 */
void X85::callIfCarry(int &cycles, Memory &mem) {
  if (statusFlags.CY) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the carry flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Transfer program control to a certain memory location only when
 * carry flag is not set.
 */
void X85::callIfOddParity(int &cycles, Memory &mem) {
  if (!statusFlags.P) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the parity flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to callIfOddParity. Transfer program control to a certain
 * memory location only when parity flag is set.
 */
void X85::callIfEvenParity(int &cycles, Memory &mem) {
  if (statusFlags.P) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the parity flag is not set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Transfer program control to a certain memory location only when
 * sign flag is not set.
 */
void X85::callIfPositive(int &cycles, Memory &mem) {
  if (!statusFlags.S) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the sign flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Similar to callIfPositive Transfer program control to a certain
 * memory location only when sign flag is set.
 */
void X85::callIfMinus(int &cycles, Memory &mem) {
  if (statusFlags.S) {
    unconditionalCall(cycles, mem);
  } else {
    // In case the sign flag is set, then just read the next byte
    // (target location), consume the required cycles and continue to
    // next instruction.
    fetchWord(cycles, mem);
  }
}

/**
 * Push the target on top os stack ( pointed by the stack pointer
 * ). As the stack grows downwards in this processor, decrement the
 * stack pointer to the new top
 */
void X85::stackPush(int &cycles, Memory &mem, const Byte &data) {
  cycles++;
  mem[SP] = data;
  SP--;
}

/**
 * Pop and return the data on top of the stack (pointed by the stack
 * pointer). As the satck grows downwards in this processor, increment
 * the stack pointer to the new top.
 */
Byte X85::stackPop(int &cycles, const Memory &mem) {
  cycles++;
  auto data = mem[SP];
  SP++;
  return data;
}

void X85::print(Memory &mem) {
  resetProgramCounter();
  int cycles = 0;
  execute(cycles,mem);
  std::cout << " executed sucessfully " << std::endl;
  std::cout << " Number of cycles : " << cycles << std::endl;

  std::cout << " A : " << std::hex << (int)getARegister() << " | ";
  std::cout << " B : " << std::hex << (int)getBRegister() << " | ";
  std::cout << " C : " << std::hex << (int)getCRegister() << " | ";
  std::cout << " D : " << std::hex << (int)getDRegister() << " | ";
  std::cout << " E : " << std::hex << (int)getERegister() << " | ";
  std::cout << " H : " << std::hex << (int)getHRegister() << " | ";
  std::cout << " L : " << std::hex << (int)getLRegister() << std::endl;

  std::cout << " CY : " << getStatusFlags().CY << " | ";
  std::cout << " P : " << getStatusFlags().P << " | ";
  std::cout << " AC : " << getStatusFlags().AC << " | ";
  std::cout << " Z : " << getStatusFlags().Z << " | ";
  std::cout << " S : " << getStatusFlags().S << std::endl;
}

int main(int argc, char* argv[]) {
  X85 microProcessor;
  short int in;
  Memory memo;
  microProcessor.reset(memo);
  if(argc>1) {
    std::ifstream ifs;
    ifs.open(argv[1],std::ifstream::in);
    std::string sin;
    while(!ifs.eof()) {
      std::getline(ifs,sin);
      in = std::stoi(sin, 0, 16);
      memo[microProcessor.getProgramCounter()] = in;
      microProcessor.incrementProgramCounter();
    }
    microProcessor.print(memo);
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