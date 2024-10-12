use crate::bus::Bus;

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

pub struct Cpu {
    pub a: u8,
    pub x: u8,
    pub y: u8,
    pub s: u8,
    pub pc: u16,
    pub status: u8,
    pub bus: Bus,
}

#[derive(Debug, PartialEq)]
struct RegisterState {
    a: u8,
    x: u8,
    y: u8,
    s: u8,
    status: u8,
}

impl Cpu {
    pub fn new(bus: Bus) -> Cpu {
        Cpu {
            a: 0,
            x: 0,
            y: 0,
            s: 0,
            pc: 0,
            status: 0,
            bus,
        }
    }

    fn reset(&mut self) {
        self.a = 0;
        self.x = 0;
        self.y = 0;
        self.s = 0xFD;
        self.status = 36;

        self.pc = self.bus.mem_read_u16(0xFFFC);
    }

    pub fn load(&mut self, program: Vec<u8>) {
        // self.memory[0x8000..(0x8000 + program.len())].copy_from_slice(&program[..]);
        self.bus.mem_write_u16(0xFFFC, 0x8000)
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();

        self.exec();
    }

    fn read(&mut self) -> u8 {
        let data = self.bus.mem_read(self.pc);
        self.pc = self.pc.wrapping_add(1);

        data
    }

    fn read_u16(&mut self) -> u16 {
        let data = self.bus.mem_read_u16(self.pc);
        self.pc = self.pc.wrapping_add(2);

        data
    }

    fn stack_push(&mut self, data: u8) {
        self.bus.mem_write(STACK_ADDR + self.s as u16, data);
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
        self.bus.mem_read(STACK_ADDR + self.s as u16)
    }

    fn stack_pull_u16(&mut self) -> u16 {
        let high = self.stack_pull() as u16;
        let low = self.stack_pull() as u16;

        (high << 8) | low
    }

    fn read_addr(&mut self, mode: AddrMode) -> u16 {
        match mode {
            AddrMode::Imm => {
                let res = self.pc;
                self.pc = self.pc.wrapping_add(1);
                res
            }
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
                self.bus.mem_read_u16(base)
            }
            AddrMode::IndX => {
                let base = self.read_u16();
                let deref = self.bus.mem_read_u16(base);
                deref.wrapping_add(self.x as u16)
            }
            AddrMode::IndY => {
                let base = self.read_u16();
                let deref = self.bus.mem_read_u16(base);
                deref.wrapping_add(self.y as u16)
            },
            AddrMode::Rel => {
                let offset = self.read() as u8;
                let abs_offset = offset & 0b01111111;

                if offset & 0b10000000 == 0 {
                    self.pc.wrapping_add(abs_offset as u16)
                } else {
                    self.pc.wrapping_sub(abs_offset as u16)
                }
            }
            // AddrMode::IndY => {
            // }
            _ => panic!("Not implemented"),
        }
    }

    fn addr_read(&mut self, mode: AddrMode) -> u8 {
        let addr = self.read_addr(mode);
        self.bus.mem_read(addr)
    }

    fn addr_write(&mut self, mode: AddrMode, data: u8) {
        let addr = self.read_addr(mode);
        self.bus.mem_write(addr, data);
    }

    fn exec(&mut self) -> (u16, RegisterState) {
        let state = (
            self.pc,
            RegisterState {
                a: self.a,
                x: self.x,
                y: self.y,
                s: self.s,
                status: self.status,
            },
        );

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

            // TODO: Shifts

            // Jump and calls
            0x4C => self.jmp(AddrMode::Abs),
            0x6C => self.jmp(AddrMode::Ind),

            0x20 => self.jsr(AddrMode::Abs),

            // Branches
            0x90 => self.bcc(AddrMode::Rel),

            0xB0 => self.bcs(AddrMode::Rel),

            0xF0 => self.beq(AddrMode::Rel),

            0xD0 => self.bne(AddrMode::Rel),

            // Status flag changes
            0x18 => self.clc(),

            0xD8 => self.cld(),

            0x58 => self.cli(),

            0xB8 => self.clv(),

            0x38 => self.sec(),

            0xF8 => self.sed(),

            0x78 => self.sei(),

            // System functions
            0xEA => (), // NOP

            _ => panic!(
                "Unknown instruction: {:#04X} at {:#06X}",
                opcode,
                self.pc - 1
            ),
        }

        state
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
        let addr = self.read_addr(mode);
        let m = self.bus.mem_read(addr).wrapping_add(1);

        self.bus.mem_write(addr, m);
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
        let addr = self.read_addr(mode);
        let m = self.bus.mem_read(addr).wrapping_sub(1);

        self.bus.mem_write(addr, m);
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
        let addr = self.read_addr(mode);
        self.pc = addr;
    }

    fn jsr(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);

        self.stack_push_u16(self.pc.wrapping_sub(1));
        self.pc = addr;
    }

    fn rts(&mut self) {
        self.pc = self.stack_pull_u16();
    }

    // Branches

    fn bcc(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);

        if !self.test_flag(StatusFlag::Carry) {
            self.pc = addr;
        }
    }

    fn bcs(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);

        if self.test_flag(StatusFlag::Carry) {
            self.pc = addr;
        }
    }

    fn beq(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);

        if self.test_flag(StatusFlag::Zero) {
            self.pc = addr;
        }
    }

    fn bne(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);

        if !self.test_flag(StatusFlag::Zero) {
            self.pc = addr;
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
    use crate::rom::{self, Rom};

    use super::*;
    use std::fs::File;
    use std::io::{BufRead, BufReader, Read};

    #[test]
    fn nestest() {
        let mut rom_data = Vec::new();
        let mut rom_file = File::open("nestest.nes").expect("Missing nestest.nes");

        rom_file.read_to_end(&mut rom_data).unwrap();

        let rom = Rom::new(&rom_data).unwrap();
        let bus = Bus::new(rom);
        let mut cpu = Cpu::new(bus);

        cpu.reset();
        cpu.pc = 0x0C000;

        eprintln!("PC: {:#4X}", cpu.pc);

        let f = File::open("nestest.log").expect("Missing nestest.log");
        let reader = BufReader::new(f);

        let testcases = reader.lines().map(|l| {
            let record = l.unwrap();

            let addr = u16::from_str_radix(&record[0..4], 16).unwrap();

            let a = u8::from_str_radix(&record[50..52], 16).unwrap();
            let x = u8::from_str_radix(&record[55..57], 16).unwrap();
            let y = u8::from_str_radix(&record[60..62], 16).unwrap();
            let status = u8::from_str_radix(&record[65..67], 16).unwrap();
            let s = u8::from_str_radix(&record[71..73], 16).unwrap();

            (addr, RegisterState { a, x, y, status, s })
        });

        for (i, test) in testcases.enumerate() {
            assert_eq!(test, cpu.exec());
            eprintln!("{} {:#X} {:?} ✅", i + 1, test.0, test.1);
        }
    }
}
