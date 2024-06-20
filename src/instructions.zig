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
//const flg_msk: u8 = 0b11000000;
const flgs = struct {
    const flg_msk: u8 = 0b00000111;
    const is_1_imm: u8 = 0b00000000; // 1st oprand; e.g. opcode 1, 2
    const is_1_ref: u8 = 0b01000000;
    const is_2_imm: u8 = 0b00000000; // 2nd oprand
    const is_2_ref: u8 = 0b10000000;
    const is_reg: u8 = 0b000;
    const is_imm: u8 = 0b001;
    const is_ref: u8 = 0b010;
};
var instruction: [@intFromEnum(InsCode.count)]*const fn (*Machine) void = undefined; // array of pointer to instruction
const InsCode = enum(usize) { push, pop, add, sub, mul, div, _and, _or, xor, ld, jmp, jz, count };
const RegIdx = enum(usize) { ip, sp, fp, gr, flag, count };

pub fn initInstructions() []*const fn (*Machine) void {
    instruction[@intFromEnum(InsCode.push)] = push;
    instruction[@intFromEnum(InsCode.pop)] = pop;
    instruction[@intFromEnum(InsCode.add)] = add;

    return &instruction;
}

fn getReg(machine: *Machine, ofs_ip: u8) *ByteWidth {
    const cpu: *Cpu = &machine.cpu;
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const target_byte = memory[ip + ofs_ip];

    switch (target_byte) {
        @intFromEnum(RegIdx.ip) => return &cpu.ip,
        @intFromEnum(RegIdx.sp) => return &cpu.sp,
        @intFromEnum(RegIdx.fp) => return &cpu.fp,
        @intFromEnum(RegIdx.gr) => return &cpu.gr,
        @intFromEnum(RegIdx.flag) => return &cpu.flag,
        else => return &cpu.flag,
    }
}

fn fetch16(mem_ref: [*]u8) u16 {
    var value: u16 = 0;

    for (mem_ref[0..2], 0..) |byte, i|
        value += shl(u16, byte, @as(u16, @intCast(i)) * 8);

    return value;
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
    const flg: u8 = memory[ip] & flgs.flg_msk;

    // TODO: if register
    // if operand is immediate value
    switch (flg) {
        flgs.is_1_imm => {
            for (0..opr_sz) |i|
                memory[sp - opr_sz + i] = memory[ip + i];
        },
        flgs.is_1_ref => {
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
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const dst: ByteWidth = if (ByteWidth == u32)
        fetch32(memory + ip + opc_sz)
    else {
        print("hoge\n", .{});
        return 0;
    };

    // TODO: if regiter
    switch (flg) {
        flgs.is_1_imm => {
            print("flg is is_1_imm\n", .{});
            return;
        },
        flgs.is_1_ref => {
            for (0..opr_sz) |i|
                memory[dst + i] = memory[sp + i];
        },
        else => {
            print("pop: unrecognized flag: {b}\n", .{flg});
            return;
        },
    }
}

// TODO: if ByteWidth != u32
fn add(machine: *Machine) void {
    var length: u8 = 1;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += length;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const dst_reg: *ByteWidth = getReg(machine, 1);

    // TODO: implement ref and imm type
    switch (flg) {
        flgs.is_reg => {
            const src: *ByteWidth = getReg(machine, 2); // pointer to destination register.
            dst_reg.* += src.*;
            length += 2;
        },
        flgs.is_imm => {
            const src = fetch16(memory + ip + 2); // imm
            dst_reg.* += src;
            length += 3;
        },
        flgs.is_ref => {
            const src: u16 = fetch16(memory + ip + 2); // ref
            dst_reg.* += fetch16(memory + src);
            length += 3;
        },
        else => {
            return;
        },
    }
}
