enum AddrMode {
    Imm,
    Zpg,
    ZpgX,
    ZpgY,
    Abs,
    AbsX,
    AbsY,
    Ind,
    IndX,
    IndY,
    Rel,
    Acc,
    Imp,
}

enum StatusFlag {
    Carry,
    Zero,
    InterrupDisable,
    Decimal,
    BreakCommand,
    Overflow,
    Negative,
}

impl Into<u8> for StatusFlag {
    fn into(self) -> u8 {
        match self {
            StatusFlag::Carry => 0b00000001,
            StatusFlag::Zero => 0b00000010,
            StatusFlag::InterrupDisable => 0b00000100,
            StatusFlag::Decimal => 0b00001000,
            StatusFlag::BreakCommand => 0b00010000,
            StatusFlag::Overflow => 0b00100000,
            StatusFlag::Negative => 0b01000000,
        }
    }
}

pub struct Cpu {
    pub a: u8,
    pub x: u8,
    pub y: u8,
    pub pc: u16,
    pub status: u8,
    memory: [u8; 0xFFFF],
}

impl Cpu {
    pub fn new() -> Cpu {
        Cpu {
            a: 0,
            x: 0,
            y: 0,
            pc: 0,
            status: 0,
            memory: [0; 0xFFFF],
        }
    }

    fn reset(&mut self) {
        self.a = 0;
        self.x = 0;
        self.status = 0;

        self.pc = self.mem_read_u16(0xFFFC);
    }

    fn load(&mut self, program: Vec<u8>) {
        self.memory[0x8000 .. (0x8000 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_u16(0xFFFC, 0x8000)
    }
    
    fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();

        self.exec();
    }

    fn mem_read(&mut self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_read_u16(&mut self, addr: u16) -> u16 {
        let low = self.mem_read(addr) as u16;
        let high = self.mem_read(addr + 1) as u16;
        
        (high << 8) | (low as u16)
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }

    fn mem_write_u16(&mut self, addr: u16, data: u16) {
        let high = (data >> 8) as u8;
        let low = (data & 0xFF) as u8;

        self.mem_write(addr, low);
        // TODO: is it really wrapping ??
        self.mem_write(addr + 1, high)
    }

    fn read(&mut self) -> u8 {
        let data = self.memory[self.pc as usize];
        self.pc = self.pc.wrapping_add(1);

        data
    }

    fn read_u16(&mut self) -> u16 {
        let data = self.mem_read_u16(self.pc);
        self.pc = self.pc.wrapping_add(2);

        data
    }


    fn addr_mode_read(&mut self, mode: AddrMode) -> u8 {
        match mode {
            AddrMode::Imm => self.read(),
            AddrMode::Zpg => self.memory[self.read_u16() as usize],
            AddrMode::ZpgX => self.memory[self.read_u16() as usize + self.x as usize],
            AddrMode::ZpgY => self.memory[self.read_u16() as usize + self.y as usize],
            _ => panic!("Cannot read from this addressing mode"),
        }
    }
    
    pub fn exec(&mut self) {
        let opcode = self.read();

        match opcode {
            0xA9 => self.lda(AddrMode::Imm),
            0xA5 => self.lda(AddrMode::Zpg),
            0xB5 => self.lda(AddrMode::ZpgX),
            0xAD => self.lda(AddrMode::Abs),
            0xBD => self.lda(AddrMode::AbsX),
            0xB9 => self.lda(AddrMode::AbsY),
            0xA1 => self.lda(AddrMode::IndX),
            0xB1 => self.lda(AddrMode::IndY),
            _ => panic!("Unknown instruction: {:#04X} at {:#06X}", opcode, self.pc - 1),
        }
    }

    fn set_flag(&mut self, flag: StatusFlag) {
        let flag_repr: u8 = flag.into();
        self.status |= flag_repr
    }

    fn unset_flag(&mut self, flag: StatusFlag) {
        let flag_repr: u8 = flag.into();
        self.status &= !flag_repr
    }

    fn update_nz(&mut self, value: u8) {
        if value == 0 {
            self.set_flag(StatusFlag::Zero)  
        } else {
            self.unset_flag(StatusFlag::Zero)
        }

        if value & 0x80 != 0 {
            self.set_flag(StatusFlag::Negative)  
        } else {
            self.unset_flag(StatusFlag::Negative)
        }
    }

    fn lda(&mut self, mode: AddrMode) {
        self.a = self.addr_mode_read(mode);
        self.update_nz(self.a)
    }

}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mem() {
        let mut cpu = Cpu::new();

        cpu.mem_write(0x69, 0x42);
        assert_eq!(cpu.mem_read(0x69), 0x42);

        cpu.mem_write_u16(0x69, 0x420);
        assert_eq!(cpu.mem_read_u16(0x69), 0x420);
    }

    #[test]
    fn test_lda() {
        let mut cpu = Cpu::new();

        println!("{}", cpu.pc);

        // Imm
        cpu.load_and_run(vec![0xA9, 0x42]);
        assert_eq!(cpu.a, 0x42);
    }

    #[test]
    fn test_flag() {
        let mut cpu = Cpu::new();

        cpu.set_flag(StatusFlag::Carry);
        assert_eq!(cpu.status, 0b00000001);

        cpu.set_flag(StatusFlag::Decimal);
        assert_eq!(cpu.status, 0b00001001);

        cpu.set_flag(StatusFlag::Negative);
        assert_eq!(cpu.status, 0b01001001);

        cpu.unset_flag(StatusFlag::Negative);
        assert_eq!(cpu.status, 0b00001001);

        cpu.unset_flag(StatusFlag::Decimal);
        assert_eq!(cpu.status, 0b00000001);

        cpu.unset_flag(StatusFlag::Carry);
        assert_eq!(cpu.status, 0b00000000);
    }

    #[test]
    fn test_update_nz() {
        let mut cpu = Cpu::new();

        cpu.update_nz(0b10000000);
        assert_eq!(cpu.status, 0b01000000);

        cpu.update_nz(0b0000000);
        assert_eq!(cpu.status, 0b00000010);
    }
}
