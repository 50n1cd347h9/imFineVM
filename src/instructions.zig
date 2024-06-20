const std = @import("std");
const print = std.debug.print;
const shl = std.math.shl;
const machine_ = @import("./machine.zig");
const machine_config = @import("./machine_config.zig");

const opr_sz = machine_config.opr_sz;
const opc_sz = machine_config.opc_sz;
const ByteWidth = machine_config.ByteWidth;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;
const Machine: type = machine_.Machine;
const Cpu: type = machine_.Cpu;
const flg_msk: u8 = 0b11000000;
const is_1_imm: u8 = 0b00000000; // 1st oprand; e.g. opcode 1, 2
const is_1_ref: u8 = 0b01000000;
const is_2_imm: u8 = 0b00000000; // 2nd oprand
const is_2_ref: u8 = 0b10000000;
var instruction: [@intFromEnum(InsCode.count)]*const fn (*Machine) void = undefined; // array of pointer to instruction
const InsCode = enum(usize) { push, pop, add, sub, mul, div, ld, jmp, jz, count };
const InsInfo: type = struct {
    length: u8, // number of bytes of instruction and operands.
    operands: u8, // number of operands that instruction takes.
};

pub fn initInstructions() []*const fn (*Machine) void {
    instruction[@intFromEnum(InsCode.push)] = push;
    instruction[@intFromEnum(InsCode.pop)] = pop;

    return &instruction;
}

fn fetch32(mem_ref: [*]u8) u32 {
    var value: u32 = 0;

    for (mem_ref[0..4], 0..) |byte, i|
        value += shl(u32, byte, @as(u32, @intCast(i)) * 8);

    return value;
}

fn write32() void {}

fn push(machine: *Machine) void {
    const length = opc_sz + opr_sz;
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
    switch (flg) {
        is_1_imm => {
            for (0..opr_sz) |i|
                memory[sp - opr_sz + i] = memory[ip + i];
        },
        is_1_ref => {
            return;
        },
        else => {
            print("pop: unrecognized flag: {b}\n", .{flg});
            return;
        },
    }
}

// TODO: error handling
// pop ref
// pop from memory[sp] and ld it to memory[ref]
fn pop(machine: *Machine) void {
    const length = opc_sz + opr_sz;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.sp += opr_sz;
        cpu.ip += length;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const sp: u32 = cpu.sp;
    const flg: u8 = memory[ip] & flg_msk;
    const dst: ByteWidth = if (ByteWidth == u32)
        fetch32(memory + ip + opc_sz)
    else {
        print("hoge\n", .{});
        return 0;
    };

    switch (flg) {
        is_1_imm => {
            print("flg is is_1_imm\n", .{});
            return;
        },
        is_1_ref => {
            for (0..opr_sz) |i|
                memory[dst + i] = memory[sp + i];
        },
        else => {
            print("pop: unrecognized flag: {b}\n", .{flg});
            return;
        },
    }
}
