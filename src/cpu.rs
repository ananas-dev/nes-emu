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

impl Cpu {
    fn read_low(&mut self) {
    }

    fn read_high(&mut self) {
    }

    fn read_temp(&mut self) {
        self.temp = self.bus.mem_read(self.pc);
        self.pc = self.pc.wrapping_add(1);
    }
}

type Instruction = fn(&mut Cpu);
type ReadInstruction = fn(&mut Cpu, u8); 
type ReadWriteInstruction = fn(&mut Cpu, u8) -> u8;
type WriteInstruction = fn(&mut Cpu) -> u8;

fn stack_push(cpu: &mut Cpu, i: Instruction) {
    match cpu.tick {
        2 => (), // dummy read
        3 => i(cpu), 
        _ => (),
    }
}

fn stack_pull_a(cpu: &mut Cpu) {
    match cpu.tick {
        2 => (), // dummy read
        3 => cpu.s = cpu.s.wrapping_add(1), 
        4 => cpu.bus.mem_write(STACK_ADDR + cpu.s as u16, cpu.a),
        _ => (),
    }
}

fn stack_pull_s(cpu: &mut Cpu) {
    match cpu.tick {
        2 => (), // dummy read
        3 => cpu.s = cpu.s.wrapping_add(1), 
        4 => cpu.bus.mem_write(STACK_ADDR + cpu.s as u16, cpu.s),
        _ => (),
    }
}

fn abs_adressing_read(cpu: &mut Cpu, i: ReadInstruction) {
    match cpu.tick {
        2 => cpu.read_temp(),
        3 => {
            let low = cpu.temp;
            cpu.read_temp();
            let high = cpu.temp;
            cpu.bus.address_bus = (high as u16) << 8 |  low as u16
        },
        4 => {
            let data = cpu.bus.mem_read(cpu.bus.address_bus);
            i(cpu, data)
        }
        _ => (),
    }
}

fn imm_addressing(cpu: &mut Cpu, i: Instruction) {
    match cpu.tick {
        2 => i(cpu),
        _ => (),
    }
}

fn acc_addressing_read_write(cpu: &mut Cpu, i: ReadWriteInstruction) {
    match cpu.tick {
        2 => cpu.a = i(cpu, cpu.a),
        _ => ()
    }
}


// Read instructions

fn lda(cpu: &mut Cpu, m: u8) {
    cpu.a = m;
    cpu.update_nz(cpu.a);
}

fn ldx(cpu: &mut Cpu, m: u8) {
    cpu.x = m;
    cpu.update_nz(cpu.x);
}

fn ldy(cpu: &mut Cpu, m: u8) {
    cpu.y = m;
    cpu.update_nz(cpu.y);
}

fn and(cpu: &mut Cpu, m: u8) {
    cpu.a &= m;
    cpu.update_nz(cpu.a);
}

fn eor(cpu: &mut Cpu, data: u8) {
    cpu.a ^= data;
    cpu.update_nz(cpu.a);
}

fn ora(cpu: &mut Cpu, data: u8) {
    cpu.a |= data;
    cpu.update_nz(cpu.a);
}

fn adc(cpu: &mut Cpu, m: u8) {
    let s = cpu.a as i32 + m as i32 + (cpu.status & 1) as i32;

    // Suspicious
    if s > 0xFF {
        cpu.set_flag(Flag::Carry);
    } else {
        cpu.clear_flag(Flag::Carry);
    }

    // https://forums.nesdev.org/viewtopic.php?t=6331
    if (cpu.a as i32 ^ s) & (m as i32 ^ s) & 0x80 != 0 {
        cpu.set_flag(Flag::Overflow);
    } else {
        cpu.clear_flag(Flag::Overflow);
    }

    // Humm
    cpu.a = s as u8;

    cpu.update_nz(cpu.a);
}

fn sbc(cpu: &mut Cpu, m: u8) {
    adc(cpu, !m);
}

fn cmp(cpu: &mut Cpu, m: u8) {
    let val = cpu.a.wrapping_sub(m);

    if cpu.a >= m {
        cpu.set_flag(Flag::Carry);
    } else {
        cpu.clear_flag(Flag::Carry);
    }

    cpu.update_nz(val);
}

fn bit(cpu: &mut Cpu, m: u8) {
    let res = cpu.a & m;

    if res == 0 {
        cpu.set_flag(Flag::Zero);
    } else {
        cpu.clear_flag(Flag::Zero);
    }

    if m & 0b01000000 != 0 {
        cpu.set_flag(Flag::Overflow)
    } else {
        cpu.clear_flag(Flag::Overflow)
    }

    if m & 0b10000000 != 0 {
        cpu.set_flag(Flag::Negative)
    } else {
        cpu.clear_flag(Flag::Negative)
    }
}

fn lax(cpu: &mut Cpu, m: u8) {
    lda(cpu, m);
    todo!()
    // tax();
}

fn nop(_cpu: &mut Cpu, _m: u8) {
}

#[repr(u8)]
enum Flag {
    Carry = 0b00000001,
    Zero = 0b00000010,
    InterruptDisable = 0b00000100,
    Decimal = 0b00001000,
    BreakCommand = 0b00010000,
    Overflow = 0b01000000,
    Negative = 0b10000000,
}

pub struct Cpu {
    pub a: u8,
    pub x: u8,
    pub y: u8,
    pub s: u8,
    pub temp: u8,
    pub pc: u16,
    pub status: u8,
    pub bus: Bus,
    pub tick: u8,
}

pub struct Op((), AddrMode, );

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
            tick: 0,
            bus,
            temp: 0,
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

        self.stack_push(high);
        self.stack_push(low);
    }

    fn stack_pull(&mut self) -> u8 {
        self.s = self.s.wrapping_add(1);
        self.bus.mem_read(STACK_ADDR + self.s as u16)
    }

    fn stack_pull_u16(&mut self) -> u16 {
        let low = self.stack_pull() as u16;
        let high = self.stack_pull() as u16;

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
                let base = self.read();
                base.wrapping_add(self.x) as u16
            }
            AddrMode::ZpgY => {
                let base = self.read();
                base.wrapping_add(self.y) as u16
            }
            AddrMode::Abs => self.read_u16(),
            AddrMode::AbsX => {
                let base = self.read_u16();
                base.wrapping_add(self.x as u16)
            }
            AddrMode::AbsY => {
                let base = self.read_u16();
                base.wrapping_add(self.y as u16)
            }
            AddrMode::Ind => {
                let base = self.read_u16();
                self.bus.mem_read_u16(base);

                let hi = (base >> 8) as u8;
                let lo = (base & 0xFF) as u8;

                let new_lo = self.bus.mem_read((hi as u16) << 8 | (lo as u16));
                let new_hi = self
                    .bus
                    .mem_read((hi as u16) << 8 | (lo.wrapping_add(1) as u16));

                (new_hi as u16) << 8 | (new_lo as u16)
            }
            AddrMode::IndX => {
                let base = self.read();
                let ptr: u8 = base.wrapping_add(self.x);

                let lo = self.bus.mem_read(ptr as u16);
                let hi = self.bus.mem_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddrMode::IndY => {
                let base = self.read();

                let lo = self.bus.mem_read(base as u16);
                let hi = self.bus.mem_read(base.wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                let deref = deref_base.wrapping_add(self.y as u16);

                deref
            }
            AddrMode::Rel => {
                let offset = self.read() as u8;

                if offset & 0b10000000 == 0 {
                    self.pc.wrapping_add((offset & 0b01111111) as u16)
                } else {
                    self.pc
                        .wrapping_sub((0b10000000 - (offset & 0b01111111)) as u16)
                }
            }
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

            0xE9 => self.sbc(AddrMode::Imm),
            0xE5 => self.sbc(AddrMode::Zpg),
            0xF5 => self.sbc(AddrMode::ZpgX),
            0xED => self.sbc(AddrMode::Abs),
            0xFD => self.sbc(AddrMode::AbsX),
            0xF9 => self.sbc(AddrMode::AbsY),
            0xE1 => self.sbc(AddrMode::IndX),
            0xF1 => self.sbc(AddrMode::IndY),

            0xC9 => self.cmp(AddrMode::Imm),
            0xC5 => self.cmp(AddrMode::Zpg),
            0xD5 => self.cmp(AddrMode::ZpgX),
            0xCD => self.cmp(AddrMode::Abs),
            0xDD => self.cmp(AddrMode::AbsX),
            0xD9 => self.cmp(AddrMode::AbsY),
            0xC1 => self.cmp(AddrMode::IndX),
            0xD1 => self.cmp(AddrMode::IndY),

            0xE0 => self.cpx(AddrMode::Imm),
            0xE4 => self.cpx(AddrMode::Zpg),
            0xEC => self.cpx(AddrMode::Abs),

            0xC0 => self.cpy(AddrMode::Imm),
            0xC4 => self.cpy(AddrMode::Zpg),
            0xCC => self.cpy(AddrMode::Abs),

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

            // Shifts

            0x0A => self.asl(AddrMode::Acc),
            0x06 => self.asl(AddrMode::Zpg),
            0x16 => self.asl(AddrMode::ZpgX),
            0x0E => self.asl(AddrMode::Abs),
            0x1E => self.asl(AddrMode::AbsX),

            0x4A => self.lsr(AddrMode::Acc),
            0x46 => self.lsr(AddrMode::Zpg),
            0x56 => self.lsr(AddrMode::ZpgX),
            0x4E => self.lsr(AddrMode::Abs),
            0x5E => self.lsr(AddrMode::AbsX),

            0x2A => self.rol(AddrMode::Acc),
            0x26 => self.rol(AddrMode::Zpg),
            0x36 => self.rol(AddrMode::ZpgX),
            0x2E => self.rol(AddrMode::Abs),
            0x3E => self.rol(AddrMode::AbsX),

            0x6A => self.ror(AddrMode::Acc),
            0x66 => self.ror(AddrMode::Zpg),
            0x76 => self.ror(AddrMode::ZpgX),
            0x6E => self.ror(AddrMode::Abs),
            0x7E => self.ror(AddrMode::AbsX),

            // Jump and calls

            0x4C => self.jmp(AddrMode::Abs),
            0x6C => self.jmp(AddrMode::Ind),

            0x20 => self.jsr(AddrMode::Abs),

            0x60 => self.rts(),

            // Branches

            0x90 => self.bcc(AddrMode::Rel),

            0xB0 => self.bcs(AddrMode::Rel),

            0xF0 => self.beq(AddrMode::Rel),

            0xD0 => self.bne(AddrMode::Rel),

            0x30 => self.bmi(AddrMode::Rel),

            0x10 => self.bpl(AddrMode::Rel),

            0x50 => self.bvc(AddrMode::Rel),

            0x70 => self.bvs(AddrMode::Rel),

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

            0x40 => self.rti(),

            // Unofficial
            0xA7 => self.lax(AddrMode::Zpg),
            0xB7 => self.lax(AddrMode::ZpgY),
            0xAF => self.lax(AddrMode::Abs),
            0xBF => self.lax(AddrMode::AbsY),
            0xA3 => self.lax(AddrMode::IndX),
            0xB3 => self.lax(AddrMode::IndY),

            0x8F => self.sax(AddrMode::Abs),
            0x87 => self.sax(AddrMode::Zpg),
            0x97 => self.sax(AddrMode::ZpgY),
            0x83 => self.sax(AddrMode::IndX),

            0xCF => self.dcp(AddrMode::Abs),
            0xDF => self.dcp(AddrMode::AbsX),
            0xDB => self.dcp(AddrMode::AbsY),
            0xC7 => self.dcp(AddrMode::Zpg),
            0xD7 => self.dcp(AddrMode::ZpgX),
            0xC3 => self.dcp(AddrMode::IndX),
            0xD3 => self.dcp(AddrMode::IndY),

            0xEF => self.isc(AddrMode::Abs),
            0xFF => self.isc(AddrMode::AbsX),
            0xFB => self.isc(AddrMode::AbsY),
            0xE7 => self.isc(AddrMode::Zpg),
            0xF7 => self.isc(AddrMode::ZpgX),
            0xE3 => self.isc(AddrMode::IndX),
            0xF3 => self.isc(AddrMode::IndY),

            0x2F => self.rla(AddrMode::Abs),
            0x3F => self.rla(AddrMode::AbsX),
            0x3B => self.rla(AddrMode::AbsY),
            0x27 => self.rla(AddrMode::Zpg),
            0x37 => self.rla(AddrMode::ZpgX),
            0x23 => self.rla(AddrMode::IndX),
            0x33 => self.rla(AddrMode::IndY),

            0x6F => self.rra(AddrMode::Abs),
            0x7F => self.rra(AddrMode::AbsX),
            0x7B => self.rra(AddrMode::AbsY),
            0x67 => self.rra(AddrMode::Zpg),
            0x77 => self.rra(AddrMode::ZpgX),
            0x63 => self.rra(AddrMode::IndX),
            0x73 => self.rra(AddrMode::IndY),

            0x0F => self.slo(AddrMode::Abs),
            0x1F => self.slo(AddrMode::AbsX),
            0x1B => self.slo(AddrMode::AbsY),
            0x07 => self.slo(AddrMode::Zpg),
            0x17 => self.slo(AddrMode::ZpgX),
            0x03 => self.slo(AddrMode::IndX),
            0x13 => self.slo(AddrMode::IndY),

            0x4F => self.sre(AddrMode::Abs),
            0x5F => self.sre(AddrMode::AbsX),
            0x5B => self.sre(AddrMode::AbsY),
            0x47 => self.sre(AddrMode::Zpg),
            0x57 => self.sre(AddrMode::ZpgX),
            0x43 => self.sre(AddrMode::IndX),
            0x53 => self.sre(AddrMode::IndY),

            0xEB => self.sbc(AddrMode::Imm),

            0x04 | 0x44 | 0x64 | 0x14 | 0x34 | 0x54 | 0x74 | 0xD4 | 0xF4 | 0x80 => {
                self.read();
            }

            0x0C | 0x1C | 0x3C | 0x5C | 0x7C | 0xDC | 0xFC => {
                self.read_u16();
            }

            0x1A | 0x3A | 0x5A | 0x7A | 0xDA | 0xFA => (),

            _ => panic!(
                "Unknown instruction: {:#04X} at {:#06X}",
                opcode,
                self.pc - 1
            ),
        }

        state
    }

    fn set_flag(&mut self, flag: Flag) {
        let flag_repr: u8 = flag as u8;
        self.status |= flag_repr;
    }

    fn clear_flag(&mut self, flag: Flag) {
        let flag_repr: u8 = flag as u8;
        self.status &= !flag_repr;
    }

    fn test_flag(&mut self, flag: Flag) -> bool {
        let flag_repr: u8 = flag as u8;
        self.status & flag_repr != 0
    }

    fn update_nz(&mut self, value: u8) {
        if value == 0 {
            self.set_flag(Flag::Zero)
        } else {
            self.clear_flag(Flag::Zero)
        }

        if value & 0b10000000 != 0 {
            self.set_flag(Flag::Negative)
        } else {
            self.clear_flag(Flag::Negative)
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
        self.stack_push(self.status | Flag::BreakCommand as u8);
    }

    fn pla(&mut self) {
        self.a = self.stack_pull();
        self.update_nz(self.a);
    }

    fn plp(&mut self) {
        self.status = 0x20 | (self.stack_pull() & 0xEF);
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
            self.set_flag(Flag::Zero);
        } else {
            self.clear_flag(Flag::Zero);
        }

        if m & 0b01000000 != 0 {
            self.set_flag(Flag::Overflow)
        } else {
            self.clear_flag(Flag::Overflow)
        }

        if m & 0b10000000 != 0 {
            self.set_flag(Flag::Negative)
        } else {
            self.clear_flag(Flag::Negative)
        }
    }

    // Should be tested
    fn adc_inner(&mut self, m: u8) {
        let s = self.a as i32 + m as i32 + (self.status & 1) as i32;

        // Suspicious
        if s > 0xFF {
            self.set_flag(Flag::Carry);
        } else {
            self.clear_flag(Flag::Carry);
        }

        // https://forums.nesdev.org/viewtopic.php?t=6331
        if (self.a as i32 ^ s) & (m as i32 ^ s) & 0x80 != 0 {
            self.set_flag(Flag::Overflow);
        } else {
            self.clear_flag(Flag::Overflow);
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
            self.set_flag(Flag::Carry);
        } else {
            self.clear_flag(Flag::Carry);
        }

        self.update_nz(val);
    }

    fn cpx(&mut self, mode: AddrMode) {
        let m = self.addr_read(mode);

        let val = self.x.wrapping_sub(m);

        if self.x >= m {
            self.set_flag(Flag::Carry);
        } else {
            self.clear_flag(Flag::Carry);
        }

        self.update_nz(val);
    }

    fn cpy(&mut self, mode: AddrMode) {
        let m = self.addr_read(mode);

        let val = self.y.wrapping_sub(m);

        if self.y >= m {
            self.set_flag(Flag::Carry);
        } else {
            self.clear_flag(Flag::Carry);
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
        self.x = self.x.wrapping_sub(1);
        self.update_nz(self.x);
    }

    fn dey(&mut self) {
        self.y = self.y.wrapping_sub(1);
        self.update_nz(self.y);
    }

    fn asl(&mut self, mode: AddrMode) {
        match mode {
            AddrMode::Acc => {
                if self.a & 0b10000000 != 0 {
                    self.set_flag(Flag::Carry);
                } else {
                    self.clear_flag(Flag::Carry);
                }

                self.a = self.a << 1;
                self.update_nz(self.a);
            }
            _ => {
                let addr = self.read_addr(mode);
                let mut m = self.bus.mem_read(addr);

                if m & 0b10000000 != 0 {
                    self.set_flag(Flag::Carry);
                } else {
                    self.clear_flag(Flag::Carry);
                }

                m = m << 1;

                self.bus.mem_write(addr, m);
                self.update_nz(m);
            }
        }
    }

    fn lsr(&mut self, mode: AddrMode) {
        match mode {
            AddrMode::Acc => {
                if self.a & 1 != 0 {
                    self.set_flag(Flag::Carry);
                } else {
                    self.clear_flag(Flag::Carry);
                }

                self.a = self.a >> 1;
                self.update_nz(self.a);
            }
            _ => {
                let addr = self.read_addr(mode);
                let mut m = self.bus.mem_read(addr);

                if m & 1 != 0 {
                    self.set_flag(Flag::Carry);
                } else {
                    self.clear_flag(Flag::Carry);
                }

                m = m >> 1;

                self.bus.mem_write(addr, m);
                self.update_nz(m);
            }
        }
    }

    fn rol(&mut self, mode: AddrMode) {
        match mode {
            AddrMode::Acc => {
                let old_carry = self.test_flag(Flag::Carry);

                if self.a & 0b10000000 != 0 {
                    self.set_flag(Flag::Carry);
                } else {
                    self.clear_flag(Flag::Carry);
                }

                self.a = self.a << 1;

                if old_carry {
                    self.a |= 0b00000001;
                }

                self.update_nz(self.a);
            }
            _ => {
                let addr = self.read_addr(mode);
                let mut m = self.bus.mem_read(addr);

                let old_carry = self.test_flag(Flag::Carry);

                if m & 0b10000000 != 0 {
                    self.set_flag(Flag::Carry);
                } else {
                    self.clear_flag(Flag::Carry);
                }

                m = m << 1;

                if old_carry {
                    m |= 0b00000001;
                }

                self.bus.mem_write(addr, m);
                self.update_nz(m);
            }
        }
    }

    fn ror(&mut self, mode: AddrMode) {
        match mode {
            AddrMode::Acc => {
                let old_carry = self.test_flag(Flag::Carry);

                if self.a & 1 != 0 {
                    self.set_flag(Flag::Carry);
                } else {
                    self.clear_flag(Flag::Carry);
                }

                self.a = self.a >> 1;

                if old_carry {
                    self.a |= 0b10000000;
                }

                self.update_nz(self.a);
            }
            _ => {
                let addr = self.read_addr(mode);
                let mut m = self.bus.mem_read(addr);

                let old_carry = self.test_flag(Flag::Carry);

                if m & 1 != 0 {
                    self.set_flag(Flag::Carry);
                } else {
                    self.clear_flag(Flag::Carry);
                }

                m = m >> 1;

                if old_carry {
                    m |= 0b10000000;
                }

                self.bus.mem_write(addr, m);
                self.update_nz(m);
            }
        }
    }

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
        self.pc = self.stack_pull_u16().wrapping_add(1);
    }

    // Branches

    fn bcc(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);

        if !self.test_flag(Flag::Carry) {
            self.pc = addr;
        }
    }

    fn bcs(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);

        if self.test_flag(Flag::Carry) {
            self.pc = addr;
        }
    }

    fn beq(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);

        if self.test_flag(Flag::Zero) {
            self.pc = addr;
        }
    }

    fn bne(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);

        if !self.test_flag(Flag::Zero) {
            self.pc = addr;
        }
    }

    fn bmi(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);

        if self.test_flag(Flag::Negative) {
            self.pc = addr;
        }
    }

    fn bpl(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);

        if !self.test_flag(Flag::Negative) {
            self.pc = addr;
        }
    }

    fn bvs(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);

        if self.test_flag(Flag::Overflow) {
            self.pc = addr;
        }
    }

    fn bvc(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);

        if !self.test_flag(Flag::Overflow) {
            self.pc = addr;
        }
    }

    fn clc(&mut self) {
        self.clear_flag(Flag::Carry);
    }

    fn cld(&mut self) {
        self.clear_flag(Flag::Decimal);
    }

    fn cli(&mut self) {
        self.clear_flag(Flag::InterruptDisable);
    }

    fn clv(&mut self) {
        self.clear_flag(Flag::Overflow);
    }

    fn sec(&mut self) {
        self.set_flag(Flag::Carry);
    }

    fn sed(&mut self) {
        self.set_flag(Flag::Decimal);
    }

    fn sei(&mut self) {
        self.set_flag(Flag::InterruptDisable);
    }

    fn brk(&mut self) {
        self.stack_push_u16(self.pc);
        self.stack_push(self.status);

        self.set_flag(Flag::BreakCommand);
    }

    fn rti(&mut self) {
        self.status = 0x20 | (self.stack_pull() & 0xEF);
        self.pc = self.stack_pull_u16();
    }

    fn lax(&mut self, mode: AddrMode) {
        self.lda(mode);
        self.tax();
    }

    fn sax(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);
        self.bus.mem_write(addr, self.a & self.x);
    }

    fn dcp(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);
        let m = self.bus.mem_read(addr).wrapping_sub(1);

        self.bus.mem_write(addr, m);
        self.update_nz(m);

        let val = self.a.wrapping_sub(m);

        if self.a >= m {
            self.set_flag(Flag::Carry);
        } else {
            self.clear_flag(Flag::Carry);
        }

        self.update_nz(val);
    }

    fn isc(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);
        let m = self.bus.mem_read(addr).wrapping_add(1);

        self.bus.mem_write(addr, m);
        // self.update_nz(m);
        self.adc_inner(!m);
    }

    fn rla(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);
        let mut m = self.bus.mem_read(addr);

        let old_carry = self.test_flag(Flag::Carry);

        if m & 0b10000000 != 0 {
            self.set_flag(Flag::Carry);
        } else {
            self.clear_flag(Flag::Carry);
        }

        m = m << 1;

        if old_carry {
            m |= 0b00000001;
        }

        self.bus.mem_write(addr, m);
        self.a &= m;
        self.update_nz(self.a);
    }

    fn rra(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);
        let mut m = self.bus.mem_read(addr);

        let old_carry = self.test_flag(Flag::Carry);

        if m & 1 != 0 {
            self.set_flag(Flag::Carry);
        } else {
            self.clear_flag(Flag::Carry);
        }

        m = m >> 1;

        if old_carry {
            m |= 0b10000000;
        }

        self.bus.mem_write(addr, m);
        self.adc_inner(m);
    }

    fn slo(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);
        let mut m = self.bus.mem_read(addr);

        if m & 0b10000000 != 0 {
            self.set_flag(Flag::Carry);
        } else {
            self.clear_flag(Flag::Carry);
        }

        m = m << 1;

        self.bus.mem_write(addr, m);
        self.a |= m;
        self.update_nz(self.a);
    }

    fn sre(&mut self, mode: AddrMode) {
        let addr = self.read_addr(mode);
        let mut m = self.bus.mem_read(addr);

        if m & 1 != 0 {
            self.set_flag(Flag::Carry);
        } else {
            self.clear_flag(Flag::Carry);
        }

        m = m >> 1;

        self.bus.mem_write(addr, m);
        self.a ^= m;
        self.update_nz(self.a);
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
            assert_eq!(cpu.exec(), test);
            eprintln!("nestest.log:{} {:#X} {:?} ✅", i + 1, test.0, test.1);
        }
    }
}
