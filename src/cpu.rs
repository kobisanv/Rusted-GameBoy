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
        (self.a as u16) << 8 | (self.f.zero as u16) << ZERO_FLAG_BYTE_POSITION
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
}

#[derive(Clone, Copy)]
enum ArithmeticTarget {
    A, B, C, D, E, H, L, BC, DE, HL
}

struct CPU {
    registers: Registers,
    flags: FlagsRegister,
    instructions: Instruction
}

impl CPU {

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

    fn execute(&mut self, instruction: Instruction) {
        match instruction {
            Instruction::ADD(target) => {
                let value = self.registers.get_register_value(target);
                let new_value = self.add(target, value);
                self.registers.set_register_value(target, new_value);
            }
            Instruction::AddHl(source) => {
                self.add_hl_rr(source);
            }
            Instruction::AddC(target) => {
                let carry = self.registers.get_register_value(target);
                let new_carry = self.adc(target, carry);
                self.registers.set_register_value(target, new_carry);
            }
            Instruction::SUB(target) => {
                let diff = self.registers.get_register_value(target);
                let new_diff = self.sub(target, diff);
                self.registers.set_register_value(target, new_diff);
            }
            Instruction::SBC(target) => {
                let diff_c = self.registers.get_register_value(target);
                let new_diffc = self.sbc(target, diff_c);
                self.registers.set_register_value(target, new_diffc);
            }
            _ => { /* TODO: Support the other instructions */}
        }
    }

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
                let (result, overflow) = self.registers.a.overflowing_add(value);
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
        self.registers.f.subtract = false;  // Clear the subtract flag as it's an addition operation
        self.registers.f.carry = did_overflow;  // Set the carry flag if there was overflow
        // Check for a half-carry condition in the addition with carry
        self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) + carry > 0xF;

        // Return the result of the addition with carry
        result_with_carry
    }

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
        }; new_value
    }

    // Perform subtraction with carry of specified value from target register
    fn sbc(&mut self, target: ArithmeticTarget, value: u8) -> u8 {
        let carry = if self.registers.f.carry { 1 } else { 0 };

        // Match the target register and get its current value and overflow information after subtracting provided value
        let (register_value, did_overflow) = match target {
            ArithmeticTarget::A => {
                let (result, overflow) = self.registers.a.overflowing_sub(value);
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
        self.registers.f.subtract = true;  // Set the subtract flag as it's a subtraction operation
        self.registers.f.carry = did_overflow;  // Set the carry flag if there was overflow
        // Check for a half-carry condition in the subtraction with carry
        self.registers.f.half_carry = (self.registers.a & 0xF) < (value & 0xF) + carry;

        // Return the result of the subtraction with carry
        result_with_carry
    }
}