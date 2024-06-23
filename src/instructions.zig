const std = @import("std");
const print = std.debug.print;
const shl = std.math.shl;
const shr = std.math.shr;
const machine_ = @import("./machine.zig");
const machine_config = @import("./machine_config.zig");

const opr_sz = machine_config.opr_sz;
const opc_sz = machine_config.opc_sz;
const ByteWidth = machine_config.ByteWidth;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;
const Machine: type = machine_.Machine;
const RegIdx: type = machine_.RegIdx;
const Cpu: type = machine_.Cpu;
const Imm32: type = machine_config.Imm32;
const Imm16: type = machine_config.Imm16;
const Imm8: type = machine_config.Imm8;
const Ref: type = machine_config.Ref;
const Reg: type = machine_config.Reg;
const flgs = struct {
    const flg_msk: u8 = 0b00000111;
    const is_reg: u8 = 0b000;
    const is_imm: u8 = 0b001;
    const is_ref: u8 = 0b010;
};
var instruction: [@intFromEnum(InsCode.count)]*const fn (*Machine) void = undefined; // array of pointer to instruction
const InsCode = enum(usize) { push, pop, add, sub, mul, div, and_, or_, xor, shl_, rld, mld, jmp, jz, count };
// mld -> memory load, rld -> register load, shl -> left shift
// st -> store; store value in gr0;

pub fn initInstructions() []*const fn (*Machine) void {
    instruction[@intFromEnum(InsCode.push)] = push;
    instruction[@intFromEnum(InsCode.pop)] = pop;
    instruction[@intFromEnum(InsCode.add)] = add;
    instruction[@intFromEnum(InsCode.sub)] = sub;
    instruction[@intFromEnum(InsCode.div)] = div;
    instruction[@intFromEnum(InsCode.and_)] = and_;
    instruction[@intFromEnum(InsCode.or_)] = or_;
    instruction[@intFromEnum(InsCode.xor)] = xor;
    instruction[@intFromEnum(InsCode.shl_)] = shl_;
    instruction[@intFromEnum(InsCode.rld)] = rld;
    instruction[@intFromEnum(InsCode.mld)] = mld;

    return &instruction;
}

fn getReg(machine: *Machine, ofs_ip: u8) Reg {
    const cpu: *Cpu = &machine.cpu;
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const target_byte = memory[ip + ofs_ip];

    switch (target_byte) {
        @intFromEnum(RegIdx.ip) => return &cpu.ip,
        @intFromEnum(RegIdx.sp) => return &cpu.sp,
        @intFromEnum(RegIdx.fp) => return &cpu.fp,
        @intFromEnum(RegIdx.gr0) => return &cpu.gr0,
        @intFromEnum(RegIdx.gr1) => return &cpu.gr1,
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

// write u16 value to [0..2]u8
fn write16(mem_ref: [*]u8, value: u16) void {
    const mask: u16 = 0x00ff;
    for (0..2) |i| {
        const byte: u16 = shr(u16, value, @as(u16, @intCast(i)) * 8) & mask;
        mem_ref[i] = @as(u8, @intCast(byte));
    }
}

// write u32 value to [0..4]u8
fn write32(mem_ref: [*]u8, value: u32) void {
    const mask: u32 = 0x000000ff;
    for (0..4) |i| {
        const byte: u32 = shr(u32, value, @as(u32, @intCast(i)) * 8) & mask;
        mem_ref[i] = @as(u8, @intCast(byte));
    }
}

fn copy32(dst: [*]u8, src: [*]u8) void {
    for (0..4) |i|
        dst[i] = src[i];
}

fn copy16(dst: [*]u8, src: [*]u8) void {
    for (0..2) |i|
        dst[i] = src[i];
}

fn push(machine: *Machine) void {
    push32(machine);
}

fn pop(machine: *Machine) void {
    pop32(machine);
}

fn add(machine: *Machine) void {
    add32(machine);
}

fn sub(machine: *Machine) void {
    sub32(machine);
}

fn mul(machine: *Machine) void {
    mul32(machine);
}

fn div(machine: *Machine) void {
    div32(machine);
}

fn and_(machine: *Machine) void {
    and32(machine);
}

fn or_(machine: *Machine) void {
    or32(machine);
}

fn xor(machine: *Machine) void {
    xor32(machine);
}

fn rld(machine: *Machine) void {
    rld32(machine);
}

fn mld(machine: *Machine) void {
    mld32(machine);
}

fn shl_(machine: *Machine) void {
    shl32(machine);
}

// push 32bit value to stack
fn push32(machine: *Machine) void {
    var ip_ofs = opc_sz;
    const stack_ofs = 4;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.sp -= stack_ofs;
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const sp: ByteWidth = cpu.sp;
    const flg: u8 = memory[ip] & flgs.flg_msk;

    switch (flg) {
        flgs.is_reg => {
            const src: Reg = getReg(machine, 1);
            write32(memory + sp - 4, src.*);
            ip_ofs += 1;
        },
        flgs.is_imm => {
            copy16(memory + sp - 2, memory + ip + opc_sz);
            ip_ofs += 2;
        },
        flgs.is_ref => {
            const src: Ref = fetch16(memory + ip + 1);
            copy32(memory + sp - 4, memory + src);
            ip_ofs += 2;
        },
        else => {
            print("pop: unrecognized flag: {b}\n", .{flg});
            return;
        },
    }
}

// TODO: error handling
fn pop32(machine: *Machine) void {
    var ip_ofs = opc_sz;
    const stack_ofs = 4;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.sp += stack_ofs;
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const sp: u32 = cpu.sp;
    const flg: u8 = memory[ip] & flgs.flg_msk;

    switch (flg) {
        flgs.is_reg => {
            const src: Imm32 = fetch32(memory + sp);
            const dst_reg: Reg = getReg(machine, 1);
            dst_reg.* = src;
            ip_ofs += 1;
        },
        flgs.is_ref => {
            const dst_ref: Ref = fetch16(memory + ip + 1);
            copy32(memory + dst_ref, memory + sp);
            ip_ofs += 2;
        },
        else => {
            print("pop: unrecognized flag: {b}\n", .{flg});
            return;
        },
    }
}

// TODO: if overflow
fn add32(machine: *Machine) void {
    var ip_ofs: u8 = opc_sz;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const dst_reg: Reg = getReg(machine, 1);

    switch (flg) {
        flgs.is_reg => {
            const src_reg: Reg = getReg(machine, 2);
            dst_reg.* += src_reg.*;
            ip_ofs += 2;
        },
        flgs.is_imm => {
            const src: Imm16 = fetch16(memory + ip + 2);
            dst_reg.* += src;
            ip_ofs += 3;
        },
        flgs.is_ref => {
            const src_ref: Ref = fetch16(memory + ip + 2);
            dst_reg.* += fetch32(memory + src_ref);
            ip_ofs += 3;
        },
        else => {
            return;
        },
    }
}

fn sub32(machine: *Machine) void {
    var ip_ofs: u8 = opc_sz;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const dst_reg: Reg = getReg(machine, 1);

    switch (flg) {
        flgs.is_reg => {
            const src_reg: Reg = getReg(machine, 2);
            dst_reg.* -= src_reg.*;
            ip_ofs += 2;
        },
        flgs.is_imm => {
            const src: Imm16 = fetch16(memory + ip + 2);
            dst_reg.* -= src;
            ip_ofs += 3;
        },
        flgs.is_ref => {
            const src_ref: Ref = fetch16(memory + ip + 2);
            dst_reg.* -= fetch32(memory + src_ref);
            ip_ofs += 3;
        },
        else => {
            return;
        },
    }
}

fn mul32(machine: *Machine) void {
    var ip_ofs: u8 = opc_sz;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const dst_reg: Reg = getReg(machine, 1);

    switch (flg) {
        flgs.is_reg => {
            const src: Reg = getReg(machine, 2);
            dst_reg.* *= src.*;
            ip_ofs += 2;
        },
        flgs.is_imm => {
            const src: Imm16 = fetch16(memory + ip + 2);
            dst_reg.* *= src;
            ip_ofs += 3;
        },
        flgs.is_ref => {
            const src_ref: Ref = fetch16(memory + ip + 2);
            dst_reg.* *= fetch32(memory + src_ref);
            ip_ofs += 3;
        },
        else => {
            return;
        },
    }
}

fn div32(machine: *Machine) void {
    var ip_ofs: u8 = opc_sz;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const dst_reg: Reg = getReg(machine, 1);

    switch (flg) {
        flgs.is_reg => {
            const src: Reg = getReg(machine, 2);
            dst_reg.* /= src.*;
            ip_ofs += 2;
        },
        flgs.is_imm => {
            const src: Imm16 = fetch16(memory + ip + 2);
            dst_reg.* /= src;
            ip_ofs += 3;
        },
        flgs.is_ref => {
            const src_ref: Ref = fetch16(memory + ip + 2);
            dst_reg.* /= fetch32(memory + src_ref);
            ip_ofs += 3;
        },
        else => {
            return;
        },
    }
}

fn and32(machine: *Machine) void {
    var ip_ofs: u8 = opc_sz;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const dst_reg: Reg = getReg(machine, 1);

    switch (flg) {
        flgs.is_reg => {
            const src: Reg = getReg(machine, 2);
            dst_reg.* &= src.*;
            ip_ofs += 2;
        },
        flgs.is_imm => {
            const src: Imm16 = fetch16(memory + ip + 2);
            dst_reg.* &= src;
            ip_ofs += 3;
        },
        flgs.is_ref => {
            const src_ref: Ref = fetch16(memory + ip + 2);
            dst_reg.* &= fetch32(memory + src_ref);
            ip_ofs += 3;
        },
        else => {
            return;
        },
    }
}

fn or32(machine: *Machine) void {
    var ip_ofs: u8 = opc_sz;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const dst_reg: Reg = getReg(machine, 1);

    switch (flg) {
        flgs.is_reg => {
            const src: Reg = getReg(machine, 2);
            dst_reg.* |= src.*;
            ip_ofs += 2;
        },
        flgs.is_imm => {
            const src: Imm16 = fetch16(memory + ip + 2);
            dst_reg.* |= src;
            ip_ofs += 3;
        },
        flgs.is_ref => {
            const src_ref: Ref = fetch16(memory + ip + 2);
            dst_reg.* |= fetch32(memory + src_ref);
            ip_ofs += 3;
        },
        else => {
            return;
        },
    }
}

fn xor32(machine: *Machine) void {
    var ip_ofs: u8 = opc_sz;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const dst_reg: Reg = getReg(machine, 1);

    switch (flg) {
        flgs.is_reg => {
            const src: Reg = getReg(machine, 2);
            dst_reg.* ^= src.*;
            ip_ofs += 2;
        },
        flgs.is_imm => {
            const src: Imm16 = fetch16(memory + ip + 2);
            dst_reg.* ^= src;
            ip_ofs += 3;
        },
        flgs.is_ref => {
            const src_ref: Ref = fetch16(memory + ip + 2);
            dst_reg.* ^= fetch32(memory + src_ref);
            ip_ofs += 3;
        },
        else => {
            return;
        },
    }
}

// load to register
fn rld32(machine: *Machine) void {
    var ip_ofs: u8 = opc_sz;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const dst_reg: Reg = getReg(machine, 1);

    switch (flg) {
        flgs.is_reg => {
            const src_reg: Reg = getReg(machine, 2);
            dst_reg.* = src_reg.*;
            ip_ofs += 2;
        },
        flgs.is_imm => {
            const src: Imm16 = fetch16(memory + ip + 2);
            dst_reg.* = src;
            ip_ofs += 3;
        },
        flgs.is_ref => {
            const src_ref: Ref = fetch16(memory + ip + 2);
            dst_reg.* = fetch32(memory + src_ref);
            ip_ofs += 3;
        },
        else => {
            return;
        },
    }
}

// load to memory pointed by register
fn mld32(machine: *Machine) void {
    var ip_ofs: u8 = opc_sz;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const dst_reg: Reg = getReg(machine, 1);

    switch (flg) {
        flgs.is_reg => {
            const src_reg: Reg = getReg(machine, 2);
            const dst_ref: Ref = fetch32(memory + dst_reg.*);
            write32(memory + dst_ref, src_reg.*);
            ip_ofs += 2;
        },
        flgs.is_imm => {
            const src: Imm16 = fetch16(memory + ip + 2);
            const dst_ref: Ref = fetch32(memory + dst_reg.*);
            write32(memory + dst_ref, src);
            ip_ofs += 3;
        },
        flgs.is_ref => {
            const src_ref: Ref = fetch16(memory + ip + 2);
            const dst_ref: Ref = fetch32(memory + dst_reg.*);
            copy32(memory + dst_ref, memory + src_ref);
            ip_ofs += 3;
        },
        else => {
            return;
        },
    }
}

fn shl32(machine: *Machine) void {
    var ip_ofs: u8 = opc_sz;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const dst_reg: Reg = getReg(machine, 1);

    switch (flg) {
        flgs.is_imm => {
            const src: Imm8 = memory[ip + 2];
            dst_reg.* = shl(u32, dst_reg.*, src);
            ip_ofs += 2;
        },
        else => {
            return;
        },
    }
}

fn pop64() void {}
fn push64() void {}
