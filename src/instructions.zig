const std = @import("std");
const print = std.debug.print;
const pow = std.math.pow;
const machine_ = @import("./machine.zig");
const machine_config = @import("./machine_config.zig");

const opr_sz = machine_config.opr_sz;
const opc_sz = machine_config.opc_sz;
const ByteWidth = machine_config.ByteWidth;
const SignedByteWidth = machine_config.SignedByteWidth;
const InsCode = machine_config.InsCode;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;
const Machine: type = machine_.Machine;
const RegIdx: type = machine_.RegIdx;
const Cpu: type = machine_.Cpu;
const Imm32: type = machine_config.Imm32;
const Imm16: type = machine_config.Imm16;
const Imm8: type = machine_config.Imm8;
const Ref: type = machine_config.Ref;
const Reg: type = machine_config.Reg;
const ext_msk: u8 = 0b00000011;
const machine_bytes: u8 = @sizeOf(ByteWidth) / 8;

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

fn getReg(cpu: *Machine.cpu, reg: u8) Reg {
    switch (reg) {
        @intFromEnum(RegIdx.ip) => return &cpu.ip,
        @intFromEnum(RegIdx.sp) => return &cpu.sp,
        @intFromEnum(RegIdx.fp) => return &cpu.fp,
        @intFromEnum(RegIdx.gr0) => return &cpu.gr0,
        @intFromEnum(RegIdx.gr1) => return &cpu.gr1,
        @intFromEnum(RegIdx.flag) => return &cpu.flag,
        else => return &cpu.flag,
    }
}

fn fetch(mem_ref: [*]u8, src_bytes: u8) void {
    var value: u32 = 0;
    var tmp: u32 = 0;
    for (0..src_bytes) |i| {
        tmp = @as(u32, @intCast(mem_ref[i])) << (i * 8);
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

fn write(mem_ref: [*]u8, value: anytype, src_bytes: u8) void {
    var byte: u16 = 0;
    for (0..src_bytes) |i| {
        byte = value >> (src_bytes - (i + 1)) << (src_bytes - 1);
        mem_ref[i] = @as(u8, @intCast(byte));
    }
    for (src_bytes..machine_bytes) |i|
        mem_ref[i] = 0x00;
}

fn ldr(machine: *Machine) void {
    ldr32(machine);
}

fn ldm(machine: *Machine) void {
    ldm32(machine);
}

fn shl_(machine: *Machine) void {
    shl32(machine);
}

fn push(machine: *Machine) void {
    var ip_ofs: u8 = 2;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
        cpu.sp -= machine_bytes;
    }
    const memory: [*]u8 = machine.memory;
    const ip: u32 = cpu.ip;
    const sp: ByteWidth = cpu.sp;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    ip_ofs +=  bytes;

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
    const ip: u32 = cpu.ip;
    const sp: ByteWidth = cpu.sp;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    ip_ofs +=  bytes;

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
    const ip: u32 = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs +=  bytes;

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
            const src_ref: Ref = (getReg(cpu. memory[ip + 2])).*;
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
    const ip: u32 = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs +=  bytes;

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
            const src_ref: Ref = (getReg(cpu. memory[ip + 2])).*;
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
    const ip: u32 = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs +=  bytes;

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
            const src_ref: Ref = (getReg(cpu. memory[ip + 2])).*;
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
    const ip: u32 = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs +=  bytes;

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
            const src_ref: Ref = (getReg(cpu. memory[ip + 2])).*;
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
    const ip: u32 = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs +=  bytes;

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
            const src_ref: Ref = (getReg(cpu. memory[ip + 2])).*;
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
    const ip: u32 = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs +=  bytes;

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
            const src_ref: Ref = (getReg(cpu. memory[ip + 2])).*;
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
    const ip: u32 = cpu.ip;
    const ext: u8 = memory[ip] & ext_msk;
    const len: u8 = memory[ip + 1] >> 5;
    const reg: u8 = memory[ip + 1] << 3 >> 5;
    const bytes: u8 = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;

    const dst_reg: Reg = getReg(cpu, reg);
    ip_ofs +=  bytes;

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
            const src_ref: Ref = (getReg(cpu. memory[ip + 2])).*;
            const src = fetch(memory + src_ref, @sizeOf(ByteWidth) / 8);
            dst_reg.* ^= src;
        },
        else => {
            return;
        },
    }
}

// load to register
fn ldr32(machine: *Machine) void {
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
fn ldm32(machine: *Machine) void {
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
            dst_reg.* = dst_reg.* << @as(u5, @intCast(src));
            ip_ofs += 2;
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
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const gr0: ByteWidth = cpu.gr0;

    switch (flg) {
        flgs.is_rel => {
            // if gr0 minus
            if (gr0 >> 31 == 0b1) {
                dst_loc = cpu.ip + opc_sz - ((gr0 << 1) >> 1);
            } else {
                dst_loc = cpu.ip + opc_sz + gr0;
            }
        },
        flgs.is_abs => {
            dst_loc = gr0;
        },
        else => print("unrecognized flag\n", .{}),
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
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const gr0: ByteWidth = cpu.gr0;

    if ((cpu.flag & 0b00) == 0b00) {
        switch (flg) {
            flgs.is_rel => {
                // if gr0 minus
                if (gr0 >> 31 == 0b1) {
                    dst_loc = cpu.ip + opc_sz - ((gr0 << 1) >> 1);
                } else {
                    dst_loc = cpu.ip + opc_sz + gr0;
                }
            },
            flgs.is_abs => {
                dst_loc = gr0;
            },
            else => print("unrecognized flag\n", .{}),
        }
    } else {
        dst_loc += cpu.ip + opc_sz;
    }
}

// jump if equal
fn jz(machine: *Machine) void {
    var dst_loc: ByteWidth = 0;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip = dst_loc;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const gr0: ByteWidth = cpu.gr0;

    if ((cpu.flag & 0b10) == 0b10) {
        switch (flg) {
            flgs.is_rel => {
                // if gr0 minus
                if (gr0 >> 31 == 0b1) {
                    dst_loc = cpu.ip + opc_sz - ((gr0 << 1) >> 1);
                } else {
                    dst_loc = cpu.ip + opc_sz + gr0;
                }
            },
            flgs.is_abs => {
                dst_loc = gr0;
            },
            else => print("unrecognized flag\n", .{}),
        }
    } else {
        dst_loc += cpu.ip + opc_sz;
    }
}

fn jl(machine: *Machine) void {
    var dst_loc: ByteWidth = 0;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip = dst_loc;
    }
    const memory: [*]u8 = machine.memory;
    const ip: ByteWidth = cpu.ip;
    const flg: u8 = memory[ip] & flgs.flg_msk;
    const gr0: ByteWidth = cpu.gr0;

    if ((cpu.flag & 0b00) == 0b00) {
        switch (flg) {
            flgs.is_rel => {
                // if gr0 signed
                if (gr0 >> 31 == 0b1) {
                    dst_loc = cpu.ip + opc_sz - ((gr0 << 1) >> 1);
                } else {
                    dst_loc = cpu.ip + opc_sz + gr0;
                }
            },
            flgs.is_abs => {
                dst_loc = gr0;
            },
            else => print("unrecognized flag\n", .{}),
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
