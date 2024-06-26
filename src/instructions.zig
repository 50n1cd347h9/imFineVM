const std = @import("std");
const builtin = std.builtin;
const print = std.debug.print;
const pow = std.math.pow;
const shl = std.math.shl;
const shr = std.math.shr;
const machine_ = @import("./machine.zig");
const machine_config = @import("./machine_config.zig");

const opr_sz = machine_config.opr_sz;
const opc_sz = machine_config.opc_sz;
const ByteWidth = machine_config.ByteWidth;
const SignedByteWidth = machine_config.SignedByteWidth;
const InsCode = machine_config.InsCode;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;
const Ext = machine_config.Ext;
const Machine: type = machine_.Machine;
const RegIdx: type = machine_.RegIdx;
const Cpu: type = machine_.Cpu;
const Imm32: type = machine_config.Imm32;
const Imm16: type = machine_config.Imm16;
const Imm8: type = machine_config.Imm8;
const Ref: type = machine_config.Ref;
const Reg: type = machine_config.Reg;
const ext_msk: u8 = 0b00000011;
const machine_bytes: u8 = @sizeOf(ByteWidth);

var instruction: [@intFromEnum(InsCode.count)]*const fn (*Machine) void = undefined; // array of pointer to instruction

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
    instruction[@intFromEnum(InsCode.ldr)] = ldr;
    instruction[@intFromEnum(InsCode.ldm)] = ldm;
    instruction[@intFromEnum(InsCode.cmp)] = cmp;
    instruction[@intFromEnum(InsCode.jmp)] = jmp;
    instruction[@intFromEnum(InsCode.jz)] = jz;
    instruction[@intFromEnum(InsCode.jg)] = jg;
    instruction[@intFromEnum(InsCode.jl)] = jl;
    instruction[@intFromEnum(InsCode.nop)] = nop;

    return &instruction;
}

fn getReg(cpu: *Cpu, reg: u8) Reg {
    switch (reg) {
        @intFromEnum(RegIdx.ip) => return &cpu.ip,
        @intFromEnum(RegIdx.sp) => return &cpu.sp,
        @intFromEnum(RegIdx.fp) => return &cpu.fp,
        @intFromEnum(RegIdx.flag) => return &cpu.flag,
        @intFromEnum(RegIdx.gr0) => return &cpu.gr0,
        @intFromEnum(RegIdx.gr1) => return &cpu.gr1,
        else => return &cpu.gr0,
    }
}

fn fetch(mem_ref: [*]u8, src_bytes: u8) ByteWidth {
    var value: ByteWidth = 0;
    for (0..src_bytes) |i| {
        const ofs: u5 = @intCast(i);
        const tmp: ByteWidth = @as(u32, @intCast(mem_ref[i])) << (ofs * 8);
        value += tmp;
    }
    return value;
}

fn copy(dst: [*]u8, src: [*]u8, src_bytes: u8) void {
    for (0..src_bytes) |i|
        dst[i] = src[i];
    for (src_bytes..machine_bytes) |i|
        dst[i] = 0x00;
}

fn write(mem_ref: [*]u8, value: ByteWidth, src_bytes: u8) void {
    const mask: ByteWidth = 0xff;
    for (0..src_bytes) |i| {
        const ofs: u5 = @intCast(i);
        const byte = shr(ByteWidth, value, ofs * 8) & mask;
        mem_ref[i] = @as(u8, @intCast(byte));
    }
    for (src_bytes..machine_bytes) |i|
        mem_ref[i] = 0x00;
}

fn push(machine: *Machine) void {
    var ip_ofs: u8 = 2;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
        cpu.sp -= machine_bytes;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const sp: ByteWidth = cpu.sp;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    ip_ofs += bytes;

    switch (ext) {
        Ext.imm => {
            copy(memory + sp - machine_bytes, memory + ip + 2, bytes);
        },
        Ext.reg => {
            const src = (getReg(cpu, memory[ip + 2])).*;
            write(memory + sp - machine_bytes, src, machine_bytes);
        },
        Ext.imm_ref => {
            const src_ref: Ref = fetch(memory + ip + 2, machine_bytes);
            copy(memory + sp - machine_bytes, memory + src_ref, machine_bytes);
        },
        Ext.reg_ref => {
            const src_ref: Ref = (getReg(cpu, memory[ip + 2])).*;
            copy(memory + sp - machine_bytes, memory + src_ref, machine_bytes);
        },
        else => {
            return;
        },
    }
}

// TODO: error handling
fn pop(machine: *Machine) void {
    var ip_ofs: u8 = 2;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
        cpu.sp += machine_bytes;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const sp: ByteWidth = cpu.sp;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    ip_ofs += bytes;

    switch (ext) {
        Ext.reg => {
            const dst_reg: Reg = getReg(cpu, memory[ip + 2]);
            const src = fetch(memory + sp, machine_bytes);
            dst_reg.* = src;
        },
        Ext.imm_ref => {
            const dst_ref: Ref = fetch(memory + ip + 2, bytes);
            copy(memory + dst_ref, memory + sp, machine_bytes);
        },
        Ext.reg_ref => {
            const dst_ref = (getReg(cpu, memory[ip + 2])).*;
            copy(memory + dst_ref, memory + sp, machine_bytes);
        },
        else => {
            return;
        },
    }
}

// TODO: if overflow
fn add(machine: *Machine) void {
    var ip_ofs: u8 = 2;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs += bytes;

    switch (ext) {
        Ext.imm => {
            const src = fetch(memory + ip + 2, bytes);
            dst_reg.* += src;
        },
        Ext.reg => {
            const src_reg: Reg = getReg(cpu, memory[ip + 2]);
            dst_reg.* += src_reg.*;
        },
        Ext.imm_ref => {
            const src_ref: Ref = fetch(memory + ip + 2, bytes);
            const src = fetch(memory + src_ref, bytes);
            dst_reg.* += src;
        },
        Ext.reg_ref => {
            const src_ref: Ref = (getReg(cpu, memory[ip + 2])).*;
            const src = fetch(memory + src_ref, machine_bytes);
            dst_reg.* += src;
        },
        else => {
            return;
        },
    }
}

fn sub(machine: *Machine) void {
    var ip_ofs: u8 = 2;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs += bytes;

    switch (ext) {
        Ext.imm => {
            const src = fetch(memory + ip + 2, bytes);
            dst_reg.* -= src;
        },
        Ext.reg => {
            const src_reg: Reg = getReg(cpu, memory[ip + 2]);
            dst_reg.* -= src_reg.*;
        },
        Ext.imm_ref => {
            const src_ref: Ref = fetch(memory + ip + 2, bytes);
            const src = fetch(memory + src_ref, len);
            dst_reg.* -= src;
        },
        Ext.reg_ref => {
            const src_ref: Ref = (getReg(cpu, memory[ip + 2])).*;
            const src = fetch(memory + src_ref, @sizeOf(ByteWidth) / 8);
            dst_reg.* -= src;
        },
        else => {
            return;
        },
    }
}

fn mul(machine: *Machine) void {
    var ip_ofs: u8 = 2;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs += bytes;

    switch (ext) {
        Ext.imm => {
            const src = fetch(memory + ip + 2, bytes);
            dst_reg.* *= src;
        },
        Ext.reg => {
            const src_reg: Reg = getReg(cpu, memory[ip + 2]);
            dst_reg.* *= src_reg.*;
        },
        Ext.imm_ref => {
            const src_ref: Ref = fetch(memory + ip + 2, bytes);
            const src = fetch(memory + src_ref, len);
            dst_reg.* *= src;
        },
        Ext.reg_ref => {
            const src_ref: Ref = (getReg(cpu, memory[ip + 2])).*;
            const src = fetch(memory + src_ref, @sizeOf(ByteWidth) / 8);
            dst_reg.* *= src;
        },
        else => {
            return;
        },
    }
}

fn div(machine: *Machine) void {
    var ip_ofs: u8 = 2;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs += bytes;

    switch (ext) {
        Ext.imm => {
            const src = fetch(memory + ip + 2, bytes);
            dst_reg.* /= src;
        },
        Ext.reg => {
            const src_reg: Reg = getReg(cpu, memory[ip + 2]);
            dst_reg.* /= src_reg.*;
        },
        Ext.imm_ref => {
            const src_ref: Ref = fetch(memory + ip + 2, bytes);
            const src = fetch(memory + src_ref, len);
            dst_reg.* /= src;
        },
        Ext.reg_ref => {
            const src_ref: Ref = (getReg(cpu, memory[ip + 2])).*;
            const src = fetch(memory + src_ref, @sizeOf(ByteWidth) / 8);
            dst_reg.* /= src;
        },
        else => {
            return;
        },
    }
}

fn and_(machine: *Machine) void {
    var ip_ofs: u8 = 2;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs += bytes;

    switch (ext) {
        Ext.imm => {
            const src = fetch(memory + ip + 2, bytes);
            dst_reg.* &= src;
        },
        Ext.reg => {
            const src_reg: Reg = getReg(cpu, memory[ip + 2]);
            dst_reg.* &= src_reg.*;
        },
        Ext.imm_ref => {
            const src_ref: Ref = fetch(memory + ip + 2, bytes);
            const src = fetch(memory + src_ref, len);
            dst_reg.* &= src;
        },
        Ext.reg_ref => {
            const src_ref: Ref = (getReg(cpu, memory[ip + 2])).*;
            const src = fetch(memory + src_ref, @sizeOf(ByteWidth) / 8);
            dst_reg.* &= src;
        },
        else => {
            return;
        },
    }
}

fn or_(machine: *Machine) void {
    var ip_ofs: u8 = 2;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs += bytes;

    switch (ext) {
        Ext.imm => {
            const src = fetch(memory + ip + 2, bytes);
            dst_reg.* |= src;
        },
        Ext.reg => {
            const src_reg: Reg = getReg(cpu, memory[ip + 2]);
            dst_reg.* |= src_reg.*;
        },
        Ext.imm_ref => {
            const src_ref: Ref = fetch(memory + ip + 2, bytes);
            const src = fetch(memory + src_ref, len);
            dst_reg.* |= src;
        },
        Ext.reg_ref => {
            const src_ref: Ref = (getReg(cpu, memory[ip + 2])).*;
            const src = fetch(memory + src_ref, @sizeOf(ByteWidth) / 8);
            dst_reg.* |= src;
        },
        else => {
            return;
        },
    }
}

fn xor(machine: *Machine) void {
    var ip_ofs: u8 = 2;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs += bytes;

    switch (ext) {
        Ext.imm => {
            const src = fetch(memory + ip + 2, bytes);
            dst_reg.* ^= src;
        },
        Ext.reg => {
            const src_reg: Reg = getReg(cpu, memory[ip + 2]);
            dst_reg.* ^= src_reg.*;
        },
        Ext.imm_ref => {
            const src_ref: Ref = fetch(memory + ip + 2, bytes);
            const src = fetch(memory + src_ref, len);
            dst_reg.* ^= src;
        },
        Ext.reg_ref => {
            const src_ref: Ref = (getReg(cpu, memory[ip + 2])).*;
            const src = fetch(memory + src_ref, @sizeOf(ByteWidth) / 8);
            dst_reg.* ^= src;
        },
        else => {
            return;
        },
    }
}

// load to register
fn ldr(machine: *Machine) void {
    var ip_ofs: u8 = 2;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs += bytes;

    switch (ext) {
        Ext.imm => {
            const src = fetch(memory + ip + 2, bytes);
            dst_reg.* = src;
        },
        Ext.reg => {
            const src_reg: Reg = getReg(cpu, memory[ip + 2]);
            dst_reg.* = src_reg.*;
        },
        Ext.imm_ref => {
            const src_ref: Ref = fetch(memory + ip + 2, bytes);
            const src = fetch(memory + src_ref, len);
            dst_reg.* = src;
        },
        Ext.reg_ref => {
            const src_ref: Ref = (getReg(cpu, memory[ip + 2])).*;
            const src = fetch(memory + src_ref, @sizeOf(ByteWidth) / 8);
            dst_reg.* = src;
        },
        else => {
            return;
        },
    }
}

// load to memory pointed by register
fn ldm(machine: *Machine) void {
    var ip_ofs: u8 = 2;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_ref: Ref = (getReg(cpu, reg)).*;
    ip_ofs += bytes;

    switch (ext) {
        Ext.imm => {
            copy(memory + dst_ref, memory + ip + 2, bytes);
        },
        Ext.reg => {
            const src_reg: Reg = getReg(cpu, memory[ip + 2]);
            write(memory + dst_ref, src_reg.*, machine_bytes);
        },
        Ext.imm_ref => {
            const src_ref: Ref = fetch(memory + ip + 2, bytes);
            copy(memory + dst_ref, memory + src_ref, machine_bytes);
        },
        Ext.reg_ref => {
            const src_ref: Ref = (getReg(cpu, memory[ip + 2])).*;
            copy(memory + dst_ref, memory + src_ref, machine_bytes);
        },
        else => {
            return;
        },
    }
}

fn shl_(machine: *Machine) void {
    var ip_ofs: u8 = 2;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs += bytes;

    switch (ext) {
        Ext.imm => {
            // src bytes is always 1.
            const src = fetch(memory + ip + 2, 1);
            dst_reg.* = dst_reg.* << @as(u5, @intCast(src));
        },
        Ext.reg => {
            const src = (getReg(cpu, memory[ip + 2])).*;
            dst_reg.* = dst_reg.* << @as(u5, @intCast(src));
        },
        else => {
            return;
        },
    }
}

fn cmp(machine: *Machine) void {
    const ip_ofs: u8 = opc_sz;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }

    const gr0: ByteWidth = cpu.gr0;
    const gr1: ByteWidth = cpu.gr1;

    if (gr0 > gr1) {
        cpu.flag &= 0b00;
    } else if (gr0 == gr1) {
        cpu.flag &= 0b01;
    } else {
        cpu.flag &= 0b10;
    }
}

fn jmp(machine: *Machine) void {
    var dst_loc: ByteWidth = 0;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip = dst_loc;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;

    switch (ext) {
        Ext.imm => {
            const dst_loc_rel: u32 = fetch(memory + ip + 2, 1);
            if (dst_loc_rel >> 31 == 0b1) {
                dst_loc = cpu.ip + opc_sz - ((dst_loc_rel << 1) >> 1);
            } else {
                dst_loc = cpu.ip + opc_sz + dst_loc_rel;
            }
        },
        else => {
            return;
        },
    }
}

// jump if greater
fn jg(machine: *Machine) void {
    var dst_loc: ByteWidth = 0;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip = dst_loc;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;

    if ((cpu.flag & 0b00) == 0b00) {
        switch (ext) {
            Ext.imm => {
                const dst_loc_rel: u32 = fetch(memory + ip + 2, 1);
                if (dst_loc_rel >> 31 == 0b1) {
                    dst_loc = cpu.ip + opc_sz - ((dst_loc_rel << 1) >> 1);
                } else {
                    dst_loc = cpu.ip + opc_sz + dst_loc_rel;
                }
            },
            else => {
                return;
            },
        }
    } else {
        dst_loc += cpu.ip + opc_sz;
    }
}

// jump if zero
fn jz(machine: *Machine) void {
    var dst_loc: ByteWidth = 0;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip = dst_loc;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;

    if ((cpu.flag & 0b01) == 0b01) {
        switch (ext) {
            Ext.imm => {
                const dst_loc_rel: u32 = fetch(memory + ip + 2, 1);
                if (dst_loc_rel >> 31 == 0b1) {
                    dst_loc = cpu.ip + opc_sz - ((dst_loc_rel << 1) >> 1);
                } else {
                    dst_loc = cpu.ip + opc_sz + dst_loc_rel;
                }
            },
            else => {
                return;
            },
        }
    } else {
        dst_loc += cpu.ip + opc_sz;
    }
}

// jump if zero
fn jl(machine: *Machine) void {
    var dst_loc: ByteWidth = 0;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip = dst_loc;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;

    if ((cpu.flag & 0b10) == 0b10) {
        switch (ext) {
            Ext.imm => {
                const dst_loc_rel: u32 = fetch(memory + ip + 2, 1);
                if (dst_loc_rel >> 31 == 0b1) {
                    dst_loc = cpu.ip + opc_sz - ((dst_loc_rel << 1) >> 1);
                } else {
                    dst_loc = cpu.ip + opc_sz + dst_loc_rel;
                }
            },
            else => {
                return;
            },
        }
    } else {
        dst_loc += cpu.ip + opc_sz;
    }
}

fn nop(machine: *Machine) void {
    const ip_ofs: u8 = opc_sz;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
}
