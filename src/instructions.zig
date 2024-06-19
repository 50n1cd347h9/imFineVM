const machine_config = @import("./machine_config.zig");
const print = @import("std").debug.print;
const machine_ = @import("./machine.zig");

const Machine: type = machine_.Machine;
const Cpu: type = machine_.Cpu;
const ByteWidth: type = machine_config.ByteWidth;
const flg_msk: u8 = 0b11000000;
const is_imm: u8 = 0b00000000;
const is_ref: u8 = 0b01000000;

const InsCode = enum(usize) { push, pop, add, sub, mul, div, ld, jmp, jz, count };
const InsInfo: type = struct {
    length: u8, // number of bytes of instruction and operands.
    operands: u8, // number of operands that instruction takes.
};

pub fn initInstructions() []*const fn (*Machine) void {
    // array of pointer to instruction
    var instruction: [@intFromEnum(InsCode.count)]*const fn (*Machine) void = undefined;

    instruction[@intFromEnum(InsCode.push)] = push;
    instruction[@intFromEnum(InsCode.pop)] = pop;

    return &instruction;
}

fn fetch32() u32 {
    // var buf: [4]u8 = undefined;
    // var value = undefined;
    //
    // return value;
}
fn write32() void {}

fn push(machine: *Machine) void {
    const opr_sz = @sizeOf(ByteWidth);
    const length = 1 + opr_sz;

    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.sp -= opr_sz;
        cpu.ip += length;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const sp: u32 = cpu.sp;
    const flg: u8 = memory[ip] & flg_msk;

    // if operand is immediate value
    if (flg == is_imm) {
        for (0..opr_sz) |i| {
            memory[sp - opr_sz + i] = memory[ip + i];
        }
    }
}

fn pop(machine: *Machine) void {
    const opr_sz = @sizeOf(ByteWidth);
    const length = 1 + opr_sz;
    _ = machine;
    _ = length;
}
