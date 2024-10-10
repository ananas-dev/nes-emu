#![feature(generic_arg_infer)]

use std::{fs::OpenOptions, iter::Cycle, ops::Add};

const STACK_ADDR: u16 = 0x0100;

enum AddrMode {
    Impl,
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
}

#[repr(u8)]
enum StatusFlag {
    Carry = 0b00000001,
    Zero = 0b00000010,
    InterruptDisable = 0b00000100,
    Decimal = 0b00001000,
    BreakCommand = 0b00010000,
    Overflow = 0b00100000,
    Negative = 0b01000000,
}

enum Instruction {
    ADC,
    AND,
    ASL,
    BCC,
    BCS,
    BEQ,
    BIT,
    BMI,
    BNE,
    BPL,
    BRK,
    BVC,
    BVS,
    CLC,
    CLD,
    CLI,
    CLV,
    CMP,
    CPX,
    CPY,
    DEC,
    DEX,
    DEY,
    EOR,
    INC,
    INX,
    INY,
    JMP,
    JSR,
    LDA,
    LDX,
    LDY,
    LSR,
    NOP,
    ORA,
    PHA,
    PHP,
    PLA,
    PLP,
    ROL,
    ROR,
    RTI,
    RTS,
    SBC,
    SEC,
    SED,
    SEI,
    STA,
    STX,
    STY,
    TAX,
    TAY,
    TSX,
    TXA,
    TXS,
    TYA,
}

struct Op(Instruction, AddrMode, fn(&mut Cpu, u16), u8);

fn brk(cpu: &mut Cpu, addr: u16) {
    cpu.stack_push_u16(cpu.pc);
    cpu.stack_push(cpu.status);

    cpu.set_flag(StatusFlag::BreakCommand);
}

fn lda(cpu: &mut Cpu, addr: u16) {
    cpu.a = cpu.mem_read(addr);
    cpu.update_nz(cpu.a);
}

const OPCODES: [Option<Op>; _] = [
    Some(Op(Instruction::BRK, AddrMode::Impl, brk, 7)),
    None,
    None,
    None,
    None,
    Some(Op(Instruction::BRK, AddrMode::Zpg, brk, 3)),
    Some(Op(Instruction::ASL, AddrMode::Zpg, brk, 5)),
    None,
    Some(OpCode::new("PHP", brk, AddrMode::Impl, 1, 3)),
    Some(OpCode::new("ORA", brk, AddrMode::Imm, 2, 2)),
    Some(OpCode::new("ASL", brk, AddrMode::Acc, 1, 2)),
    None,
    None,
    Some(OpCode::new("ORA", brk, AddrMode::Abs, 3, 2)),
    Some(OpCode::new("ASL", brk, AddrMode::Acc, 1, 2)),
];

#[derive(Debug)]
pub struct Cpu {
    pub a: u8,
    pub x: u8,
    pub y: u8,
    pub s: u8,
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
            s: 0,
            pc: 0,
            status: 0,
            memory: [0; 0xFFFF],
        }
    }

    fn reset(&mut self) {
        self.a = 0;
        self.x = 0;
        self.y = 0;
        self.s = 0xFF;
        self.status = 0;

        self.pc = self.mem_read_u16(0xFFFC);
    }

    pub fn load(&mut self, program: Vec<u8>) {
        self.memory[0x8000..(0x8000 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_u16(0xFFFC, 0x8000)
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
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

        (high << 8) | low
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

    fn stack_push(&mut self, data: u8) {
        self.mem_write(STACK_ADDR + self.s as u16, data);
        self.s = self.s.wrapping_sub(1);
    }

    fn stack_push_u16(&mut self, data: u16) {
        let high = (data >> 8) as u8;
        let low = (data & 0xFF) as u8;

        self.stack_push(low);
        self.stack_push(high);
    }

    fn stack_pull(&mut self) -> u8 {
        self.s = self.s.wrapping_add(1);
        self.mem_read(STACK_ADDR + self.s as u16)
    }

    fn stack_pull_u16(&mut self) -> u16 {
        let high = self.stack_pull() as u16;
        let low = self.stack_pull() as u16;

        (high << 8) | low
    }

    fn addr(&mut self, mode: AddrMode) -> u16 {
        match mode {
            AddrMode::Imm => self.pc,
            AddrMode::Zpg => self.read() as u16,
            AddrMode::ZpgX => {
                let base = self.read() as u16;
                base.wrapping_add(self.x as u16)
            }
            AddrMode::ZpgY => {
                let base = self.read() as u16;
                base.wrapping_add(self.y as u16)
            }
            AddrMode::Abs => self.read_u16(),
            AddrMode::AbsX => {
                let base = self.read_u16();
                base.wrapping_add(self.x as u16)
            }
            AddrMode::AbsY => {
                let base = self.read_u16();
                base.wrapping_add(self.x as u16)
            }
            AddrMode::Ind => {
                let base = self.read_u16();
                self.mem_read_u16(base)
            }
            AddrMode::IndX => {
                let base = self.read_u16();
                self.mem_read_u16(base)
            }
            // AddrMode::IndY => {
            // }
            _ => panic!("Not implemented"),
        }
    }

    fn addr_read(&mut self, mode: AddrMode) -> u8 {
        let addr = self.addr(mode);
        self.mem_read(addr)
    }

    fn addr_write(&mut self, mode: AddrMode, data: u8) {
        let addr = self.addr(mode);
        self.mem_write(addr, data);
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

            0xA2 => self.ldx(AddrMode::Imm),
            0xA6 => self.ldx(AddrMode::Zpg),
            0xB6 => self.ldx(AddrMode::ZpgY),
            0xAE => self.ldx(AddrMode::Abs),
            0xBE => self.ldx(AddrMode::AbsY),

            0xA0 => self.ldy(AddrMode::Imm),
            0xA4 => self.ldy(AddrMode::Zpg),
            0xB4 => self.ldy(AddrMode::ZpgX),
            0xAC => self.ldy(AddrMode::Abs),
            0xBC => self.ldy(AddrMode::AbsX),

            0x85 => self.sta(AddrMode::Zpg),
            0x95 => self.sta(AddrMode::ZpgX),
            0x8D => self.sta(AddrMode::Abs),
            0x9D => self.sta(AddrMode::AbsX),
            0x99 => self.sta(AddrMode::AbsY),
            0x81 => self.sta(AddrMode::IndX),
            0x91 => self.sta(AddrMode::IndY),

            0x86 => self.stx(AddrMode::Zpg),
            0x96 => self.stx(AddrMode::ZpgY),
            0x8E => self.stx(AddrMode::Abs),

            0x84 => self.sty(AddrMode::Zpg),
            0x94 => self.sty(AddrMode::ZpgX),
            0x8C => self.sty(AddrMode::Abs),

            0xAA => self.tax(),

            0xA8 => self.tay(),

            0x8A => self.txa(),

            0x98 => self.tya(),

            0xBA => self.tsx(),

            0x9A => self.txs(),

            0x48 => self.pha(),

            0x08 => self.php(),

            0x68 => self.pla(),

            0x28 => self.plp(),

            0x29 => self.and(AddrMode::Imm),
            0x25 => self.and(AddrMode::Zpg),
            0x35 => self.and(AddrMode::ZpgX),
            0x2D => self.and(AddrMode::Abs),
            0x3D => self.and(AddrMode::AbsX),
            0x39 => self.and(AddrMode::AbsY),
            0x21 => self.and(AddrMode::IndX),
            0x31 => self.and(AddrMode::IndY),

            0x49 => self.eor(AddrMode::Imm),
            0x45 => self.eor(AddrMode::Zpg),
            0x55 => self.eor(AddrMode::ZpgX),
            0x4D => self.eor(AddrMode::Abs),
            0x5D => self.eor(AddrMode::AbsX),
            0x59 => self.eor(AddrMode::AbsY),
            0x41 => self.eor(AddrMode::IndX),
            0x51 => self.eor(AddrMode::IndY),

            0x09 => self.ora(AddrMode::Imm),
            0x05 => self.ora(AddrMode::Zpg),
            0x15 => self.ora(AddrMode::ZpgX),
            0x0D => self.ora(AddrMode::Abs),
            0x1D => self.ora(AddrMode::AbsX),
            0x19 => self.ora(AddrMode::AbsY),
            0x01 => self.ora(AddrMode::IndX),
            0x11 => self.ora(AddrMode::IndY),

            0x24 => self.bit(AddrMode::Zpg),
            0x2C => self.bit(AddrMode::Abs),

            0x69 => self.adc(AddrMode::Imm),
            0x65 => self.adc(AddrMode::Zpg),
            0x75 => self.adc(AddrMode::ZpgX),
            0x6D => self.adc(AddrMode::Abs),
            0x7D => self.adc(AddrMode::AbsX),
            0x79 => self.adc(AddrMode::AbsY),
            0x61 => self.adc(AddrMode::IndX),
            0x71 => self.adc(AddrMode::IndY),

            // ...
            0xE6 => self.inc(AddrMode::Zpg),
            0xF6 => self.inc(AddrMode::ZpgX),
            0xEE => self.inc(AddrMode::Abs),
            0xFE => self.inc(AddrMode::AbsX),

            0xE8 => self.inx(),

            0xC8 => self.iny(),

            0xC6 => self.dec(AddrMode::Zpg),
            0xD6 => self.dec(AddrMode::ZpgX),
            0xCE => self.dec(AddrMode::Abs),
            0xDE => self.dec(AddrMode::AbsX),

            0xCA => self.dex(),

            0x88 => self.dey(),

            0xEA => (), // NOP

            _ => panic!(
                "Unknown instruction: {:#04X} at {:#06X}",
                opcode,
                self.pc - 1
            ),
        }
    }

    fn set_flag(&mut self, flag: StatusFlag) {
        let flag_repr: u8 = flag as u8;
        self.status |= flag_repr
    }

    fn clear_flag(&mut self, flag: StatusFlag) {
        let flag_repr: u8 = flag as u8;
        self.status &= !flag_repr
    }

    fn test_flag(&mut self, flag: StatusFlag) -> bool {
        let flag_repr: u8 = flag as u8;
        self.status & flag_repr != 0
    }

    fn update_nz(&mut self, value: u8) {
        if value == 0 {
            self.set_flag(StatusFlag::Zero)
        } else {
            self.clear_flag(StatusFlag::Zero)
        }

        if value & 0x80 != 0 {
            self.set_flag(StatusFlag::Negative)
        } else {
            self.clear_flag(StatusFlag::Negative)
        }
    }

    fn lda(&mut self, mode: AddrMode) {
        self.a = self.addr_read(mode);
        self.update_nz(self.a);
    }

    fn ldx(&mut self, mode: AddrMode) {
        self.x = self.addr_read(mode);
        self.update_nz(self.x);
    }

    fn ldy(&mut self, mode: AddrMode) {
        self.y = self.addr_read(mode);
        self.update_nz(self.y);
    }

    fn sta(&mut self, mode: AddrMode) {
        self.addr_write(mode, self.a)
    }

    fn stx(&mut self, mode: AddrMode) {
        self.addr_write(mode, self.x)
    }

    fn sty(&mut self, mode: AddrMode) {
        self.addr_write(mode, self.y)
    }

    fn tax(&mut self) {
        self.x = self.a;
        self.update_nz(self.x);
    }

    fn tay(&mut self) {
        self.y = self.a;
        self.update_nz(self.y);
    }

    fn txa(&mut self) {
        self.a = self.x;
        self.update_nz(self.a);
    }

    fn tya(&mut self) {
        self.a = self.y;
        self.update_nz(self.a);
    }

    fn tsx(&mut self) {
        self.x = self.s;
        self.update_nz(self.x);
    }

    fn txs(&mut self) {
        self.s = self.x;
    }

    fn pha(&mut self) {
        self.stack_push(self.a);
    }

    fn php(&mut self) {
        self.stack_push(self.status);
    }

    fn pla(&mut self) {
        self.a = self.stack_pull();
        self.update_nz(self.a);
    }

    fn plp(&mut self) {
        self.status = self.stack_pull();
    }

    fn and(&mut self, mode: AddrMode) {
        self.a &= self.addr_read(mode);
        self.update_nz(self.a);
    }

    fn eor(&mut self, mode: AddrMode) {
        self.a ^= self.addr_read(mode);
        self.update_nz(self.a);
    }

    fn ora(&mut self, mode: AddrMode) {
        self.a |= self.addr_read(mode);
        self.update_nz(self.a);
    }

    fn bit(&mut self, mode: AddrMode) {
        let m = self.addr_read(mode);
        let res = self.a & m;

        if res == 0 {
            self.set_flag(StatusFlag::Zero);
        } else {
            self.clear_flag(StatusFlag::Zero);
        }

        if m & 0b01000000 != 0 {
            self.set_flag(StatusFlag::Overflow)
        } else {
            self.clear_flag(StatusFlag::Overflow)
        }

        if m & 0b10000000 != 0 {
            self.set_flag(StatusFlag::Negative)
        } else {
            self.clear_flag(StatusFlag::Negative)
        }
    }

    // Should be tested
    fn adc_inner(&mut self, m: u8) {
        let s = self.a as i32 + m as i32 + (self.status & 1) as i32;

        // Suspicious
        if s > 0xFF {
            self.set_flag(StatusFlag::Carry);
        } else {
            self.clear_flag(StatusFlag::Carry);
        }

        // https://forums.nesdev.org/viewtopic.php?t=6331
        if (self.a as i32 ^ s) & (m as i32 ^ s) & 0x80 != 0 {
            self.set_flag(StatusFlag::Overflow);
        } else {
            self.clear_flag(StatusFlag::Overflow);
        }

        // Humm
        self.a = s as u8;

        self.update_nz(self.a);
    }

    fn adc(&mut self, mode: AddrMode) {
        let m = self.addr_read(mode);
        self.adc_inner(m);
    }

    fn sbc(&mut self, mode: AddrMode) {
        let m = self.addr_read(mode);
        self.adc_inner(!m);
    }

    fn cmp(&mut self, mode: AddrMode) {
        let m = self.addr_read(mode);

        let val = self.a.wrapping_sub(m);

        if self.a >= m {
            self.set_flag(StatusFlag::Carry);
        }

        self.update_nz(val);
    }

    fn cmx(&mut self, mode: AddrMode) {
        let m = self.addr_read(mode);

        let val = self.x.wrapping_sub(m);

        if self.x >= m {
            self.set_flag(StatusFlag::Carry);
        }

        self.update_nz(val);
    }

    fn cmy(&mut self, mode: AddrMode) {
        let m = self.addr_read(mode);

        let val = self.y.wrapping_sub(m);

        if self.y >= m {
            self.set_flag(StatusFlag::Carry);
        }

        self.update_nz(val);
    }

    fn inc(&mut self, mode: AddrMode) {
        let addr = self.addr(mode);
        let m = self.mem_read(addr).wrapping_add(1);

        self.mem_write(addr, m);
        self.update_nz(m);
    }

    fn inx(&mut self) {
        self.x = self.x.wrapping_add(1);
        self.update_nz(self.x);
    }

    fn iny(&mut self) {
        self.y = self.y.wrapping_add(1);
        self.update_nz(self.y);
    }

    fn dec(&mut self, mode: AddrMode) {
        let addr = self.addr(mode);
        let m = self.mem_read(addr).wrapping_sub(1);

        self.mem_write(addr, m);
        self.update_nz(m);
    }

    fn dex(&mut self) {
        self.x = self.x.wrapping_add(1);
        self.update_nz(self.x);
    }

    fn dey(&mut self) {
        self.y = self.y.wrapping_sub(1);
        self.update_nz(self.y);
    }

    fn asl(&mut self) {}

    fn jmp(&mut self, mode: AddrMode) {
        let addr = self.addr(mode);
        self.pc = addr;
    }

    fn jsr(&mut self, mode: AddrMode) {
        let addr = self.addr(mode);

        self.stack_push_u16(self.pc.wrapping_sub(1));
        self.pc = addr;
    }

    fn rts(&mut self) {
        self.pc = self.stack_pull_u16();
    }

    // Branches

    fn bcc(&mut self, mode: AddrMode) {
        let offset: i8 = todo!();

        if !self.test_flag(StatusFlag::Carry) {
            if offset > 0 {
                self.pc = self.pc.wrapping_add(offset as u16)
            } else {
                self.pc = self.pc.wrapping_sub((-offset) as u16)
            }
        }
    }

    fn bcs(&mut self, mode: AddrMode) {
        let offset: i8 = todo!();

        if self.test_flag(StatusFlag::Carry) {
            if offset > 0 {
                self.pc = self.pc.wrapping_add(offset as u16)
            } else {
                self.pc = self.pc.wrapping_sub((-offset) as u16)
            }
        }
    }

    fn beq(&mut self, mode: AddrMode) {
        let offset: i8 = todo!();

        if self.test_flag(StatusFlag::Zero) {
            if offset > 0 {
                self.pc = self.pc.wrapping_add(offset as u16)
            } else {
                self.pc = self.pc.wrapping_sub((-offset) as u16)
            }
        }
    }

    fn clc(&mut self) {
        self.clear_flag(StatusFlag::Carry);
    }

    fn cld(&mut self) {
        self.clear_flag(StatusFlag::Decimal);
    }

    fn cli(&mut self) {
        self.clear_flag(StatusFlag::InterruptDisable);
    }

    fn clv(&mut self) {
        self.clear_flag(StatusFlag::Overflow);
    }

    fn sec(&mut self) {
        self.set_flag(StatusFlag::Carry);
    }

    fn sed(&mut self) {
        self.set_flag(StatusFlag::Decimal);
    }

    fn sei(&mut self) {
        self.set_flag(StatusFlag::InterruptDisable);
    }

    fn brk(&mut self) {
        self.stack_push_u16(self.pc);
        self.stack_push(self.status);

        self.set_flag(StatusFlag::BreakCommand);
    }

    fn rti(&mut self) {
        self.status = self.stack_pull();
        self.pc = self.stack_pull_u16();
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
    fn test_addr_read() {
        let mut cpu = Cpu::new();

        // Immediate
        cpu.load_and_run(vec![0xA9, 0x42]);
        assert_eq!(cpu.a, 0x42);

        cpu.mem_write(0x0069, 0x42);

        // Zero page
        cpu.load_and_run(vec![0xA5, 0x69]);
        assert_eq!(cpu.a, 0x42);
    }

    #[test]
    fn test_lda() {
        let mut cpu = Cpu::new();

        cpu.load_and_run(vec![0xA9, 0x42]);
        assert_eq!(cpu.a, 0x42);
    }

    #[test]
    fn test_ldx() {
        let mut cpu = Cpu::new();

        cpu.load_and_run(vec![0xA2, 0x42]);
        assert_eq!(cpu.x, 0x42);
    }

    #[test]
    fn test_ldy() {
        let mut cpu = Cpu::new();

        cpu.load_and_run(vec![0xA0, 0x42]);
        assert_eq!(cpu.y, 0x42);
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

        assert_eq!(cpu.test_flag(StatusFlag::BreakCommand), false);

        assert_eq!(cpu.test_flag(StatusFlag::Negative), true);

        cpu.clear_flag(StatusFlag::Negative);
        assert_eq!(cpu.status, 0b00001001);

        cpu.clear_flag(StatusFlag::Decimal);
        assert_eq!(cpu.status, 0b00000001);

        cpu.clear_flag(StatusFlag::Carry);
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
