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

    fn read_byte(&mut self) -> u8 {
        let data = self.memory[self.pc as usize];
        self.pc = self.pc.wrapping_add(1);

        data
    }

    fn read_word(&mut self) -> u16 {
        let low = self.memory[self.pc as usize] as u16;
        let high = self.memory[self.pc as usize] as u16;
        
        self.pc = self.pc.wrapping_add(2);

        (high << 8) | (low as u16)
    }


    fn addr_mode_read(&mut self, mode: AddrMode) -> u8 {
        match mode {
            AddrMode::Imm => self.read_byte(),
            AddrMode::Zpg => self.memory[self.read_word() as usize],
            AddrMode::ZpgX => self.memory[self.read_word() as usize + self.x as usize],
            AddrMode::ZpgY => self.memory[self.read_word() as usize + self.y as usize],
            _ => panic!("Cannot read from this addressing mode"),
        }
    }
    
    pub fn exec(&mut self) {
        let opcode = self.read_byte();

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

        println!("{}", value);
        println!("{}", value & 0x80);

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
