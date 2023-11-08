use crate::cpu::Cpu;

mod cpu;

fn main() {
    let mut cpu = Cpu::new();

    cpu.exec();

    println!("Hello, world!");
}
