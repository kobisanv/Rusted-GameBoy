// Reading GameBoy Documentation and Implementing the Registers
struct Registers {
    a: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    f: FlagsRegister, // flags register
    h: u8,
    l: u8,
}

impl Registers {
    // Get the 16-bit value of the AF register (combining A and F registers)
    fn get_af(&self) -> u16 {
        // getter (self is not mutable as we are only accessing value)
        // Convert A to a 16-bit unsigned integer and shift it 8 positions to the left (high bit)
        // Combine it with the F register using bitwise OR (low bit)
        (self.a as u16) << 8
            | (self.f.zero as u16) << ZERO_FLAG_BYTE_POSITION
            | (self.f.subtract as u16) << SUBTRACT_FLAG_BYTE_POSITION
            | (self.f.half_carry as u16) << HALF_CARRY_FLAG_BYTE_POSITION
            | (self.f.carry as u16) << CARRY_FLAG_BYTE_POSITION
    }

    // Set the value of the AF register with a 16-bit value
    fn set_af(&mut self, value: u16) {
        // setter
        // Extract the high 8 bits (A) and assign it to the A register
        self.a = ((value & 0xFF00) >> 8) as u8;
        // Extract the low 8 bits (F) and assign it to the F register
        self.f = FlagsRegister::from(value as u8);
    }

    // Combining BC registers
    fn get_bc(&self) -> u16 {
        (self.b as u16) << 8 | self.c as u16
    }

    fn set_bc(&mut self, value: u16) {
        self.b = ((value & 0xFF00) >> 8) as u8;
        self.c = (value & 0xFF) as u8;
    }

    // Combining DE registers
    fn get_de(&self) -> u16 {
        (self.d as u16) << 8 | self.e as u16
    }

    fn set_de(&mut self, value: u16) {
        self.d = ((value & 0xFF00) >> 8) as u8;
        self.e = (value & 0xFF) as u8;
    }

    // Combining HL registers
    fn get_hl(&self) -> u16 {
        (self.h as u16) << 8 | self.l as u16
    }

    fn set_hl(&mut self, value: u16) {
        self.h = ((value & 0xFF00) >> 8) as u8;
        self.l = (value & 0xFF) as u8;
    }

    // Get the value of a specific register
    fn get_register_value(&self, target: ArithmeticTarget) -> u8 {
        match target {
            ArithmeticTarget::A => self.a,
            ArithmeticTarget::B => self.b,
            ArithmeticTarget::C => self.c,
            ArithmeticTarget::D => self.d,
            ArithmeticTarget::E => self.e,
            ArithmeticTarget::H => self.h,
            ArithmeticTarget::L => self.l,
            _ => unimplemented!(),
        }
    }

    // Set the value of a specific register
    fn set_register_value(&mut self, target: ArithmeticTarget, value: u8) {
        match target {
            ArithmeticTarget::A => self.a = value,
            ArithmeticTarget::B => self.b = value,
            ArithmeticTarget::C => self.c = value,
            ArithmeticTarget::D => self.d = value,
            ArithmeticTarget::E => self.e = value,
            ArithmeticTarget::H => self.h = value,
            ArithmeticTarget::L => self.l = value,
            _ => unimplemented!(),
        }
    }
}

/* Register F is special as it is the flags register. -> stores the first four bits as the instruction that is happening (zero, subtract, half-carry, carry)
*  The lower four bits or the low bit are always 0s and CPU only writes to high bit when anything happens (it "flags" the different states)
*  Bit 7: Zero | Bit 6: Subtraction | Bit 5: Half-Carry | Bit 4: Carry
*  ┌-> Carry
 ┌-+> Subtraction
 | |
1111 0000
| |
└-+> Zero
  └-> Half Carry
*/

// Constants representing the bit positions of flags in the FlagsRegister byte
const ZERO_FLAG_BYTE_POSITION: u8 = 7;
const SUBTRACT_FLAG_BYTE_POSITION: u8 = 6;
const HALF_CARRY_FLAG_BYTE_POSITION: u8 = 5;
const CARRY_FLAG_BYTE_POSITION: u8 = 4;

// FlagsRegister struct representing CPU flags
struct FlagsRegister {
    zero: bool,
    subtract: bool,
    half_carry: bool,
    carry: bool,
}

// Implement conversion from FlagsRegister to u8 (byte)
impl From<FlagsRegister> for u8 {
    fn from(flag: FlagsRegister) -> u8 {
        // Convert boolean flags to 1 or 0, shift them to their respective positions,
        // and combine them using bitwise OR to create a single u8 byte
        (if flag.zero { 1 } else { 0 }) << ZERO_FLAG_BYTE_POSITION
            | (if flag.subtract { 1 } else { 0 }) << SUBTRACT_FLAG_BYTE_POSITION
            | (if flag.half_carry { 1 } else { 0 }) << HALF_CARRY_FLAG_BYTE_POSITION
            | (if flag.carry { 1 } else { 0 }) << CARRY_FLAG_BYTE_POSITION
    }
}

// Implement conversion from u8 (byte) to FlagsRegister
impl From<u8> for FlagsRegister {
    fn from(byte: u8) -> Self {
        // Extract individual flags from high bit by right-shifting to transport it to low bit (right-shift byte by 7, 6, 5, 4) and masking (& 0b1) (bitwise AND)
        // masking allows only the least significant bit or the right-most bit which in this case this is 'byte' to be remaining while removing every other bit before
        let zero = ((byte >> ZERO_FLAG_BYTE_POSITION) & 0b1) != 0;
        let subtract = ((byte >> SUBTRACT_FLAG_BYTE_POSITION) & 0b1) != 0;
        let half_carry = ((byte >> HALF_CARRY_FLAG_BYTE_POSITION) & 0b1) != 0;
        let carry = ((byte >> CARRY_FLAG_BYTE_POSITION) & 0b1) != 0;

        // Create a FlagsRegister with the extracted flag values
        FlagsRegister {
            zero,
            subtract,
            half_carry,
            carry,
        }
    }
}

// Instructions for the Registers
enum Instruction {
    ADD(ArithmeticTarget),
    AddHl(ArithmeticTarget),
    AddC(ArithmeticTarget),
    SUB(ArithmeticTarget),
    SBC(ArithmeticTarget),
    AND(ArithmeticTarget),
    OR(ArithmeticTarget),
    XOR(ArithmeticTarget),
    CP(ArithmeticTarget),
    INC(ArithmeticTarget),
    DEC(ArithmeticTarget),
    CCF(ArithmeticTarget),
    SCF(ArithmeticTarget),
    RRA(ArithmeticTarget),
    RLA(ArithmeticTarget),
    RRCA(ArithmeticTarget),
    RRLA(ArithmeticTarget),
    CPL(ArithmeticTarget),
    BIT(ArithmeticTarget, u8),
    RESET(ArithmeticTarget, u8),
    SET(ArithmeticTarget, u8),
    SRL(ArithmeticTarget),
    RR(ArithmeticTarget),
    RL(ArithmeticTarget),
    RRC(ArithmeticTarget),
    RLC(ArithmeticTarget),
    SRA(ArithmeticTarget),
    SLA(ArithmeticTarget),
    SWAP(ArithmeticTarget),
    INCR(IncDecTarget),
    RLCR(PrefixTarget),
}

#[derive(Clone, Copy)]
enum ArithmeticTarget {
    A,
    B,
    C,
    D,
    E,
    H,
    L,
    BC,
    DE,
    HL,
}

enum IncDecTarget { BC }
enum PrefixTarget { B }

struct CPU {
    registers: Registers,
    flags: FlagsRegister,
    instructions: Instruction,
    pc: u16,
    bus: MemoryBus,
}

struct MemoryBus {
    memory: [u8; 0xFFFF]
}

impl MemoryBus {
    fn read_byte(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }
}

impl Instruction {
    fn from_byte(byte: u8, prefixed: bool) -> Option<Instruction> {
        if prefixed {
            Instruction::from_byte_prefixed(byte)
        } else {
            Instruction::from_byte_not_prefixed(byte)
        }
    }

    fn from_byte_prefixed(byte: u8) -> Option<Instruction> {
        match byte {
            _ => /* TODO: Add mapping for rest of instructions */ None
        }
    }

    fn from_byte_not_prefixed(byte: u8) -> Option<Instruction> {
        match byte {
            _ => /* TODO: Add mapping for rest of instructions */ None
        }
    }
}

impl CPU {

    fn step(&mut self) {
        let mut instruction_byte = self.bus.read_byte(self.pc);
        let prefixed = instruction_byte == 0xCB;
        if prefixed {
            instruction_byte = self.bus.read_byte(self.pc + 1);
        }

        let next_pc = if let Some(instruction) = Instruction::from_byte(instruction_byte, prefixed) {
            self.execute(instruction)
        } else {
            let description = format!("0x{}{:x}", if prefixed { "cb" } else { "" }, instruction_byte);
            panic!("Unknown instruction found for: {}", description)
        };

        self.pc = next_pc;

    }

    // Function to get value of specific 16-bit register pair
    fn get_register_pair_value_name(&self, target: ArithmeticTarget) -> u16 {
        // match the target register pair and return its value
        match target {
            ArithmeticTarget::BC => self.registers.get_bc(),
            ArithmeticTarget::DE => self.registers.get_de(),
            ArithmeticTarget::HL => self.registers.get_hl(),
            _ => unimplemented!(),
        }
    }

    fn execute(&mut self, instruction: Instruction) -> u16 {
        match instruction {
            Instruction::ADD(target) => {
                let value = self.registers.get_register_value(target);
                let new_value = self.add(target, value);
                self.registers.set_register_value(target, new_value);
                self.pc.wrapping_add(1)
            }
            Instruction::AddHl(source) => {
                self.add_hl_rr(source);
                self.pc.wrapping_add(1)
            }
            Instruction::AddC(target) => {
                let carry = self.registers.get_register_value(target);
                let new_carry = self.adc(target, carry);
                self.registers.set_register_value(target, new_carry);
                self.pc.wrapping_add(1)
            }
            Instruction::SUB(target) => {
                let diff = self.registers.get_register_value(target);
                let new_diff = self.sub(target, diff);
                self.registers.set_register_value(target, new_diff);
                self.pc.wrapping_add(1)
            }
            Instruction::SBC(target) => {
                let diff_c = self.registers.get_register_value(target);
                let new_diffc = self.sbc(target, diff_c);
                self.registers.set_register_value(target, new_diffc);
                self.pc.wrapping_add(1)
            }
            Instruction::AND(target) => {
                self.and(target);
                self.pc.wrapping_add(1)
            }
            Instruction::OR(target) => {
                self.or(target);
                self.pc.wrapping_add(1)
            }
            Instruction::XOR(target) => {
                self.xor(target);
                self.pc.wrapping_add(1)
            }
            Instruction::CP(target) => {
                let value = self.registers.get_register_value(target);
                self.cp(target, value);
                self.pc.wrapping_add(1)
            }
            Instruction::INC(target) => {
                self.inc(target);
                self.pc.wrapping_add(1)
            }
            Instruction::DEC(target) => {
                self.dec(target);
                self.pc.wrapping_add(1)
            }
            Instruction::CCF(target) => {
                self.ccf();
                self.pc.wrapping_add(1)
            }
            Instruction::SCF(target) => {
                self.scf();
                self.pc.wrapping_add(1)
            }
            Instruction::RRA(target) => {
                self.rra();
                self.pc.wrapping_add(1)
            }
            Instruction::RLA(target) => {
                self.rla();
                self.pc.wrapping_add(1)
            }
            Instruction::RRCA(target) => {
                self.rrca();
                self.pc.wrapping_add(1)
            }
            Instruction::RRLA(target) => {
                self.rrla();
                self.pc.wrapping_add(1)
            }
            Instruction::CPL(target) => {
                self.cpl();
                self.pc.wrapping_add(1)
            }
            Instruction::BIT(target, Bit) => {
                self.bit(target, Bit);
                self.pc.wrapping_add(1)
            }
            Instruction::RESET(target, Bit) => {
                self.reset(target, Bit);
                self.pc.wrapping_add(1)
            }
            Instruction::SET(target, Bit) => {
                self.set(target, Bit);
                self.pc.wrapping_add(1)
            }
            Instruction::SRL(target) => {
                self.srl(target);
                self.pc.wrapping_add(1)
            }
            Instruction::RR(target) => {
                self.rr(target);
                self.pc.wrapping_add(1)
            }
            Instruction::RL(target) => {
                self.rl(target);
                self.pc.wrapping_add(1)
            }
            Instruction::RRC(target) => {
                self.rrc(target);
                self.pc.wrapping_add(1)
            }
            Instruction::RLC(target) => {
                self.rlc(target);
                self.pc.wrapping_add(1)
            }
            Instruction::SRA(target) => {
                self.sra(target);
                self.pc.wrapping_add(1)
            }
            Instruction::SLA(target) => {
                self.sla(target);
                self.pc.wrapping_add(1)
            }
            Instruction::SWAP(target) => {
                self.swap(target);
                self.pc.wrapping_add(1)
            }
            _ => unimplemented!(),
        }
    }

    // Perform addition
    fn add(&mut self, target: ArithmeticTarget, value: u8) -> u8 {
        let (new_value, did_overflow) = match target {
            ArithmeticTarget::A => {
                // Perform addition of the value in register A and the provided value
                let (new_value, overflow) = self.registers.a.overflowing_add(value);

                // Set the zero flag based on whether the result is zero
                self.registers.f.zero = new_value == 0;

                // Clear the subtract flag (not performing subtraction)
                self.registers.f.subtract = false;

                // Set the carry flag based on overflow in the addition
                self.registers.f.carry = overflow;

                // Check for a half-carry condition in the addition
                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) > 0xF;

                // Return a tuple containing the result of the addition and overflow information
                (new_value, overflow)
            }
            ArithmeticTarget::B => {
                let (new_value, overflow) = self.registers.b.overflowing_add(value);
                self.registers.f.zero = new_value == 0;
                self.registers.f.subtract = false;
                self.registers.f.carry = overflow;
                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) > 0xF;
                (new_value, overflow)
            }
            ArithmeticTarget::C => {
                let (new_value, overflow) = self.registers.c.overflowing_add(value);
                self.registers.f.zero = new_value == 0;
                self.registers.f.subtract = false;
                self.registers.f.carry = overflow;
                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) > 0xF;
                (new_value, overflow)
            }
            ArithmeticTarget::D => {
                let (new_value, overflow) = self.registers.d.overflowing_add(value);
                self.registers.f.zero = new_value == 0;
                self.registers.f.subtract = false;
                self.registers.f.carry = overflow;
                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) > 0xF;
                (new_value, overflow)
            }
            ArithmeticTarget::E => {
                let (new_value, overflow) = self.registers.e.overflowing_add(value);
                self.registers.f.zero = new_value == 0;
                self.registers.f.subtract = false;
                self.registers.f.carry = overflow;
                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) > 0xF;
                (new_value, overflow)
            }
            ArithmeticTarget::H => {
                let (new_value, overflow) = self.registers.h.overflowing_add(value);
                self.registers.f.zero = new_value == 0;
                self.registers.f.subtract = false;
                self.registers.f.carry = overflow;
                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) > 0xF;
                (new_value, overflow)
            }
            ArithmeticTarget::L => {
                let (new_value, overflow) = self.registers.l.overflowing_add(value);
                self.registers.f.zero = new_value == 0;
                self.registers.f.subtract = false;
                self.registers.f.carry = overflow;
                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) > 0xF;
                (new_value, overflow)
            }
            _ => {
                // Handle other cases or return a default value as needed
                self.registers.f.zero = false;
                self.registers.f.subtract = false;
                self.registers.f.carry = false;
                self.registers.f.half_carry = false;
                (0, false)
            }
        };
        new_value
    }

    // Perform addition but only with HL register
    fn add_hl_rr(&mut self, source: ArithmeticTarget) {
        let hl_value = self.registers.get_hl(); // get current value in HL register pair
        let rr_value = self.registers.get_register_value(source); // get the value of the specified 16-bit register pair (rr)
        let (result, did_overflow) = hl_value.overflowing_add(rr_value as u16); // perform the 16-bit addition and check for overflow
        self.registers.set_hl(result); // update the HL register pair with the result of the addition
        self.registers.f.subtract = false; // reset the subtract flag
        self.registers.f.carry = did_overflow; // set the carry flag if there was overflow in the addition
        self.registers.f.half_carry = (hl_value & 0xFFF) + (rr_value as u16 & 0xFFF) > 0xFFF;
        // set the half-carry flag if adding the lower 12 bits of HL and rr together result in a value bigger than 0xFFF
    }

    // Perform addition with carry of the specified value to the target register
    fn adc(&mut self, target: ArithmeticTarget, value: u8) -> u8 {
        // Determine the carry value based on the carry flag in the status register
        let carry = if self.registers.f.carry { 1 } else { 0 };

        // Match the target register and get its current value and overflow status after adding the provided value
        let (register_value, did_overflow) = match target {
            ArithmeticTarget::A => {
                let (result, overflow) = self.registers.a.overflowing_add(value); // returns tuple with bool telling if overflow occurred
                (result, overflow)
            }
            ArithmeticTarget::B => {
                let (result, overflow) = self.registers.b.overflowing_add(value);
                (result, overflow)
            }
            ArithmeticTarget::C => {
                let (result, overflow) = self.registers.c.overflowing_add(value);
                (result, overflow)
            }
            ArithmeticTarget::D => {
                let (result, overflow) = self.registers.d.overflowing_add(value);
                (result, overflow)
            }
            ArithmeticTarget::E => {
                let (result, overflow) = self.registers.e.overflowing_add(value);
                (result, overflow)
            }
            ArithmeticTarget::H => {
                let (result, overflow) = self.registers.h.overflowing_add(value);
                (result, overflow)
            }
            ArithmeticTarget::L => {
                let (result, overflow) = self.registers.l.overflowing_add(value);
                (result, overflow)
            }
            // Handle other cases or return default values for unknown targets
            _ => {
                // Update status flags to indicate an invalid operation
                self.registers.f.zero = false;
                self.registers.f.subtract = false;
                self.registers.f.carry = false;
                self.registers.f.half_carry = false;
                // Return 0 as a placeholder value for an invalid operation
                return 0;
            }
        };

        // Perform addition with carry to the register value
        let (result_with_carry, overflow_with_carry) = register_value.overflowing_add(carry);

        // Get the carry value again for further calculations
        let carry = if self.registers.f.carry { 1 } else { 0 };

        // Perform addition with carry to the A register value
        let (result, overflow) = self.registers.a.overflowing_add(value);
        let (result_with_carry, overflow_with_carry) = result.overflowing_add(carry);

        // Determine if there was overflow in the addition with carry
        let did_overflow = overflow || overflow_with_carry;

        // Update status flags based on the result of the addition with carry
        self.registers.f.zero = result_with_carry == 0;
        self.registers.f.subtract = false; // Clear the subtract flag as it's an addition operation
        self.registers.f.carry = did_overflow; // Set the carry flag if there was overflow
                                               // Check for a half-carry condition in the addition with carry
        self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) + carry > 0xF;

        // Return the result of the addition with carry
        result_with_carry
    }

    // Perform subtraction
    fn sub(&mut self, target: ArithmeticTarget, value: u8) -> u8 {
        let (new_value, did_overflow) = match target {
            ArithmeticTarget::A => {
                // perform subtraction of value in register A and provided value
                let (new_value, overflow) = self.registers.a.overflowing_sub(value);

                // set the zero flag using whether the result is zero
                self.registers.f.zero = new_value == 0;

                // set subtract flag to true
                self.registers.f.subtract = true;

                // set carry flag based on overflow in subtraction
                self.registers.f.carry = overflow;

                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);

                // return tuple containing result of subtraction and overflow information
                (new_value, overflow)
            }
            ArithmeticTarget::B => {
                let (new_value, overflow) = self.registers.b.overflowing_sub(value);
                self.registers.f.zero = new_value == 0;
                self.registers.f.subtract = true;
                self.registers.f.carry = overflow;
                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);
                (new_value, overflow)
            }
            ArithmeticTarget::C => {
                let (new_value, overflow) = self.registers.c.overflowing_sub(value);
                self.registers.f.zero = new_value == 0;
                self.registers.f.subtract = true;
                self.registers.f.carry = overflow;
                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);
                (new_value, overflow)
            }
            ArithmeticTarget::D => {
                let (new_value, overflow) = self.registers.d.overflowing_sub(value);
                self.registers.f.zero = new_value == 0;
                self.registers.f.subtract = true;
                self.registers.f.carry = overflow;
                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);
                (new_value, overflow)
            }
            ArithmeticTarget::E => {
                let (new_value, overflow) = self.registers.e.overflowing_sub(value);
                self.registers.f.zero = new_value == 0;
                self.registers.f.subtract = true;
                self.registers.f.carry = overflow;
                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);
                (new_value, overflow)
            }
            ArithmeticTarget::H => {
                let (new_value, overflow) = self.registers.h.overflowing_sub(value);
                self.registers.f.zero = new_value == 0;
                self.registers.f.subtract = true;
                self.registers.f.carry = overflow;
                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);
                (new_value, overflow)
            }
            ArithmeticTarget::L => {
                let (new_value, overflow) = self.registers.l.overflowing_sub(value);
                self.registers.f.zero = new_value == 0;
                self.registers.f.subtract = true;
                self.registers.f.carry = overflow;
                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);
                (new_value, overflow)
            }
            _ => {
                // Handle other cases or return a default value as needed
                self.registers.f.zero = false;
                self.registers.f.subtract = false;
                self.registers.f.carry = false;
                self.registers.f.half_carry = false;
                (0, false)
            }
        };
        new_value
    }

    // Perform subtraction with carry of specified value from target register
    fn sbc(&mut self, target: ArithmeticTarget, value: u8) -> u8 {
        let carry = if self.registers.f.carry { 1 } else { 0 };

        // Match the target register and get its current value and overflow information after subtracting provided value
        let (register_value, did_overflow) = match target {
            ArithmeticTarget::A => {
                let (result, overflow) = self.registers.a.overflowing_sub(value); // returns tuple with bool telling if overflow occurred
                (result, overflow)
            }
            ArithmeticTarget::B => {
                let (result, overflow) = self.registers.a.overflowing_sub(value);
                (result, overflow)
            }
            ArithmeticTarget::C => {
                let (result, overflow) = self.registers.c.overflowing_sub(value);
                (result, overflow)
            }
            ArithmeticTarget::D => {
                let (result, overflow) = self.registers.d.overflowing_sub(value);
                (result, overflow)
            }
            ArithmeticTarget::E => {
                let (result, overflow) = self.registers.e.overflowing_sub(value);
                (result, overflow)
            }
            ArithmeticTarget::H => {
                let (result, overflow) = self.registers.h.overflowing_sub(value);
                (result, overflow)
            }
            ArithmeticTarget::L => {
                let (result, overflow) = self.registers.l.overflowing_sub(value);
                (result, overflow)
            }
            // Handle other cases or return default values for unknown targets
            _ => {
                // Update status flags to indicate an invalid operation
                self.registers.f.zero = false;
                self.registers.f.subtract = false;
                self.registers.f.carry = false;
                self.registers.f.half_carry = false;
                // Return 0 as a placeholder value for an invalid operation
                return 0;
            }
        };
        // Perform subtraction with carry from the register value
        let (result_with_carry, overflow_with_carry) = register_value.overflowing_sub(carry);

        // Get the carry value again for further calculations
        let carry = if self.registers.f.carry { 1 } else { 0 };

        // Perform subtraction with carry from the A register value
        let (result, overflow) = self.registers.a.overflowing_sub(value);
        let (result_with_carry, overflow_with_carry) = result.overflowing_sub(carry);

        // Determine if there was overflow in the subtraction with carry
        let did_overflow = overflow || overflow_with_carry;

        // Update status flags based on the result of the subtraction with carry
        self.registers.f.zero = result_with_carry == 0;
        self.registers.f.subtract = true; // Set the subtract flag as it's a subtraction operation
        self.registers.f.carry = did_overflow; // Set the carry flag if there was overflow
                                               // Check for a half-carry condition in the subtraction with carry
        self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF) + carry;

        // Return the result of the subtraction with carry
        result_with_carry
    }

    // Perform a bitwise AND operation between value in specified register and value in A register
    fn and(&mut self, target: ArithmeticTarget) {
        // Get value from specified register
        let value = self.registers.get_register_value(target);

        // Perform AND operation between value in target and value in A
        let result = self.registers.a & value;

        // Update A register with result of AND operation
        self.registers.a = result;

        // Update status flags based on result
        self.registers.f.zero = result == 0; // Set the zero flag if the result is zero
        self.registers.f.subtract = false; // Clear the subtract flag
        self.registers.f.carry = false; // Clear the carry flag
        self.registers.f.half_carry = false; // Clear the half-carry flag
    }

    // Perform a bitwise OR operation between value in specified register and value in A register
    fn or(&mut self, target: ArithmeticTarget) {
        // Get value from specified register
        let value = self.registers.get_register_value(target);

        // Perform OR operation between value in target and value in A
        let result = self.registers.a | value;

        // Update A register with result of OR operation
        self.registers.a = result;

        // Update status flags based on result
        self.registers.f.zero = result == 0; // Set the zero flag if the result is zero
        self.registers.f.subtract = false; // Clear the subtract flag
        self.registers.f.carry = false; // Clear the carry flag
        self.registers.f.half_carry = false; // Clear the half-carry flag
    }

    // Perform a bitwise XOR operation between value in specified register and value in A register
    fn xor(&mut self, target: ArithmeticTarget) {
        // Get value from specified register
        let value = self.registers.get_register_value(target);

        // Perform XOR operation between value in target and value in A
        let result = self.registers.a ^ value;

        // Update A register with result of XOR operation
        self.registers.a = result;

        // Update status flags based on result
        self.registers.f.zero = result == 0; // Set the zero flag if the result is zero
        self.registers.f.subtract = false; // Clear the subtract flag
        self.registers.f.carry = false; // Clear the carry flag
        self.registers.f.half_carry = false; // Clear the half-carry flag
    }

    // Perform comparison with value in specified register with value in A register without modifying A register
    fn cp(&mut self, target: ArithmeticTarget, value: u8) {
        match target {
            ArithmeticTarget::A => {
                // perform subtraction of value in A register and provided value
                let (_, overflow) = self.registers.a.overflowing_add(value);

                // set zero flag using whether the result is zero
                self.registers.f.zero = self.registers.a == value;

                // set subtract flag to true
                self.registers.f.subtract = true;

                // set carry flag based on overflow in subtraction
                self.registers.f.carry = overflow;

                // Half Carry is set if adding the lower nibbles of the value and the register
                // together result in a value bigger than 0xF.
                self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);
            }
            ArithmeticTarget::B => {
                let (_, overflow) = self.registers.b.overflowing_add(value);
                self.registers.f.zero = self.registers.b == value;
                self.registers.f.subtract = true;
                self.registers.f.carry = overflow;
                self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);
            }
            ArithmeticTarget::C => {
                let (_, overflow) = self.registers.c.overflowing_add(value);
                self.registers.f.zero = self.registers.c == value;
                self.registers.f.subtract = true;
                self.registers.f.carry = overflow;
                self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);
            }
            ArithmeticTarget::D => {
                let (_, overflow) = self.registers.d.overflowing_add(value);
                self.registers.f.zero = self.registers.d == value;
                self.registers.f.subtract = true;
                self.registers.f.carry = overflow;
                self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);
            }
            ArithmeticTarget::E => {
                let (_, overflow) = self.registers.e.overflowing_add(value);
                self.registers.f.zero = self.registers.e == value;
                self.registers.f.subtract = true;
                self.registers.f.carry = overflow;
                self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);
            }
            ArithmeticTarget::H => {
                let (_, overflow) = self.registers.h.overflowing_add(value);
                self.registers.f.zero = self.registers.h == value;
                self.registers.f.subtract = true;
                self.registers.f.carry = overflow;
                self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);
            }
            ArithmeticTarget::L => {
                let (_, overflow) = self.registers.l.overflowing_add(value);
                self.registers.f.zero = self.registers.l == value;
                self.registers.f.subtract = true;
                self.registers.f.carry = overflow;
                self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF);
            }
            _ => {
                self.registers.f.zero = false;
                self.registers.f.subtract = false;
                self.registers.f.carry = false;
                self.registers.f.half_carry = false;
            }
        }
    }

    // Increment the value in a specific register by 1
    fn inc(&mut self, target: ArithmeticTarget) {
        let (register_value, _) = match target {
            ArithmeticTarget::A => (&mut self.registers.a, false),
            ArithmeticTarget::B => (&mut self.registers.b, false),
            ArithmeticTarget::C => (&mut self.registers.c, false),
            ArithmeticTarget::D => (&mut self.registers.d, false),
            ArithmeticTarget::E => (&mut self.registers.e, false),
            ArithmeticTarget::H => (&mut self.registers.h, false),
            ArithmeticTarget::L => (&mut self.registers.l, false),
            _ => {
                return;
            }
        };

        *register_value = register_value.wrapping_add(1);

        // Update status flags
        self.registers.f.zero = false;
        self.registers.f.subtract = false;
        self.registers.f.carry = false;
        self.registers.f.half_carry = false;
    }

    // Decrement the value in a specific register by 1
    fn dec(&mut self, target: ArithmeticTarget) {
        let (register_value, _) = match target {
            ArithmeticTarget::A => (&mut self.registers.a, false),
            ArithmeticTarget::B => (&mut self.registers.b, false),
            ArithmeticTarget::C => (&mut self.registers.c, false),
            ArithmeticTarget::D => (&mut self.registers.d, false),
            ArithmeticTarget::E => (&mut self.registers.e, false),
            ArithmeticTarget::H => (&mut self.registers.h, false),
            ArithmeticTarget::L => (&mut self.registers.l, false),
            _ => {
                return;
            }
        };

        *register_value = register_value.wrapping_sub(1);

        // Update status flags
        self.registers.f.zero = false;
        self.registers.f.subtract = false;
        self.registers.f.carry = false;
        self.registers.f.half_carry = false;
    }

    // Complement carry flag
    fn ccf(&mut self) {
        self.registers.f.subtract = false;
        self.registers.f.half_carry = false;
        self.registers.f.carry = !self.registers.f.carry;
    }

    // Set carry flag
    fn scf(&mut self) {
        self.registers.f.subtract = false;
        self.registers.f.half_carry = false;
        self.registers.f.carry = true;
    }

    // Rotate right A register through carry flag
    fn rra(&mut self) {
        // extract current carry flag state
        let carry = if self.registers.f.carry { 0x80 } else { 0 };

        // determine new carry flag based on least significant bit
        let new_carry = self.registers.a & 0x01 != 0;

        // perform rotation to right
        self.registers.a = (self.registers.a >> 1) | carry;

        // update status flags
        self.registers.f.carry = new_carry;
        self.registers.f.zero = self.registers.a == 0;
        self.registers.f.subtract = false;
        self.registers.f.half_carry = false;
    }

    // Rotate left A register through carry flag
    fn rla(&mut self) {
        // extract current carry flag state
        let carry = if self.registers.f.carry { 0x80 } else { 0 };

        // determine new carry flag based on most significant bit
        let new_carry = self.registers.a & 0x80 != 0;

        // perform rotation to left
        self.registers.a = (self.registers.a << 1) | carry;

        // update status flags
        self.registers.f.carry = new_carry;
        self.registers.f.zero = self.registers.a == 0;
        self.registers.f.subtract = false;
        self.registers.f.half_carry = false;
    }

    // Rotate right A register not using the carry flag
    fn rrca(&mut self) {
        // determine new carry flag based on least significant bit
        let new_carry = self.registers.a & 0x01 != 0;

        // perform rotation to right
        self.registers.a = (self.registers.a >> 1) | (self.registers.a << 7);

        // update status flags
        self.registers.f.carry = new_carry;
        self.registers.f.zero = self.registers.a == 0;
        self.registers.f.subtract = false;
        self.registers.f.half_carry = false;
    }

    // Rotate left A register not using the carry flag
    fn rrla(&mut self) {
        // determine new carry flag based on most significant bit
        let new_carry = self.registers.a & 0x80 != 0;

        // perform the rotation to the left
        self.registers.a = (self.registers.a << 1) | (self.registers.a >> 7);

        // update status flags
        self.registers.f.carry = new_carry;
        self.registers.f.zero = self.registers.a == 0;
        self.registers.f.subtract = false;
        self.registers.f.half_carry = false;
    }

    // Toggle every bit of A register
    fn cpl(&mut self) {
        // toggle
        self.registers.a = !self.registers.a;

        // set flags
        self.registers.f.subtract = true;
        self.registers.f.half_carry = true; // since operation involves borrow in lower bit, half-carry true
    }

    // Test to see if specific bit of register is set
    fn bit(&mut self, target: ArithmeticTarget, bit: u8) {
        // get value of specified register
        let value = self.registers.get_register_value(target);

        // test if specific bit is set
        let is_set = value & (1 << bit) != 0;

        // update flags
        self.registers.f.zero = !is_set;
        self.registers.f.subtract = false;
        self.registers.f.half_carry = true;
    }

    // Set a specific bit of specific register to 0
    fn reset(&mut self, target: ArithmeticTarget, bit: u8) {
        // get value of specified register
        let mut value = self.registers.get_register_value(target);

        // clear specified bit
        value &= !(1 << bit);

        // update register with modified value
        self.registers.set_register_value(target, value);
    }

    // Set a specific bit of specific register to 1
    fn set(&mut self, target: ArithmeticTarget, bit: u8) {
        // get value of specified register
        let mut value = self.registers.get_register_value(target);

        // set specified bit
        value |= 1 << bit;

        // update register with modified value
        self.registers.set_register_value(target, value);
    }

    // Bit shift a specific register right by 1
    fn srl(&mut self, target: ArithmeticTarget) {
        // get value of specified register
        let mut value = self.registers.get_register_value(target);

        // perform logical shift right
        let result = value >> 1;

        // set flags based on result
        self.set_flags_after_shift(target, result);

        // store result back in register
        self.registers.set_register_value(target, result);
    }

    // Bit rotate a specific register right by 1 through carry flag
    fn rr(&mut self, target: ArithmeticTarget) {
        // get value of specified register
        let mut value = self.registers.get_register_value(target);

        // perform rotate right through carry
        let result = (value >> 1) | (self.registers.f.carry as u8).wrapping_shl(7);

        // set flags based on result
        self.set_flags_after_shift(target, result);

        // store result back in register
        self.registers.set_register_value(target, result);
    }

    // Bit rotate a specific register left by 1 through carry flag
    fn rl(&mut self, target: ArithmeticTarget) {
        // get value of specified register
        let value = self.registers.get_register_value(target);

        // perform rotate left through carry
        let result = (value << 1) | (self.registers.f.carry as u8);

        // set flags based on result
        self.set_flags_after_shift(target, result);

        // store result back in register
        self.registers.set_register_value(target, result);
    }

    // Bit rotate a specific register right by 1 without carry flag
    fn rrc(&mut self, target: ArithmeticTarget) {
        // get value of specified register
        let value = self.registers.get_register_value(target);

        // perform rotate right without carry
        let result = (value >> 1) | (value << 7);

        // set flags based on result
        self.set_flags_after_shift(target, result);

        // store result back in register
        self.registers.set_register_value(target, result);
    }

    // Bit rotate a specific register left by 1 without carry flag
    fn rlc(&mut self, target: ArithmeticTarget) {
        // get value of specified register
        let value = self.registers.get_register_value(target);

        // perform rotate left without carry
        let result = (value << 1) | (value >> 7);

        // set flags based on result
        self.set_flags_after_shift(target, result);

        // store result back in register
        self.registers.set_register_value(target, result);
    }

    // Arithmetic shift a specific register right by 1
    fn sra(&mut self, target: ArithmeticTarget) {
        // get value of specified register
        let value = self.registers.get_register_value(target);

        // perform arithmetic shift right (preserving sign bit)
        let result = ((value as i8) >> 1) as u8; // if u8, most significant bit will be replaced with 0

        // set flags based on result
        self.set_flags_after_shift(target, result);

        // store result back in register
        self.registers.set_register_value(target, result);
    }

    // Arithmetic shift a specific register left by 1
    fn sla(&mut self, target: ArithmeticTarget) {
        // get value of specified register
        let value = self.registers.get_register_value(target);

        // perform arithmetic shift left
        let result = value << 1;

        // set flags based on result
        self.set_flags_after_shift(target, result);

        // store result back in register
        self.registers.set_register_value(target, result);
    }

    // swap upper and lower nibbles
    fn swap(&mut self, target: ArithmeticTarget) {
        // get value of specified register
        let value = self.registers.get_register_value(target);

        // perform nibble swap
        let result = (value << 4) | (value >> 4);

        // update register with swapped value
        self.registers.set_register_value(target, result);

        // update flags
        self.registers.f.zero = result == 0;
        self.registers.f.subtract = false;
        self.registers.f.carry = false;
        self.registers.f.half_carry = false;
    }

    // Helper function to set flags after shift instructions
    fn set_flags_after_shift(&mut self, target: ArithmeticTarget, result: u8) {
        // Update zero flag
        self.registers.f.zero = result == 0;

        // Update subtract and carry flags
        self.registers.f.subtract = false;

        self.registers.f.carry = (result & 0x01) != 0; // bitwise AND to extract least significant bit using bitmask 0x01 and retains bit at pos 0

        // Half-carry is not affected by shift instructions
        self.registers.f.half_carry = false;
    }
}
