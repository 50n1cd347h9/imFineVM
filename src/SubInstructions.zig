const machine_config = @import("machine_config.zig");
const Machine = @import("Machine.zig");
const shr = @import("std").math.shr;

const ImFineMac: type = Machine.ImFineMac;
const RegIdx: type = machine_config.RegIdx;
const Cpu: type = Machine.Cpu;
const Ref: type = machine_config.Ref;
const Reg: type = machine_config.Reg;
const ByteWidth = machine_config.ByteWidth;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;
const MACHINE_BYTES: u8 = machine_config.MACHINE_BYTES;

extern var _self_instructions: *@import("Instructions.zig");

pub inline fn getReg(reg: u8) Reg {
    const self = _self_instructions;
    return switch (reg) {
        @intFromEnum(RegIdx.ip) => &self.cpu.ip,
        @intFromEnum(RegIdx.sp) => &self.cpu.sp,
        @intFromEnum(RegIdx.fp) => &self.cpu.fp,
        @intFromEnum(RegIdx.flag) => &self.cpu.flag,
        @intFromEnum(RegIdx.gr0) => &self.cpu.gr0,
        @intFromEnum(RegIdx.gr1) => &self.cpu.gr1,
        else => &self.cpu.gr0,
    };
}

pub inline fn fetch(ref: Ref, length: u8) ByteWidth {
    var value: ByteWidth = 0;
    for (0..length) |i| {
        const ofs: u5 = @intCast(i);
        const tmp: ByteWidth = @as(
            u32,
            @intCast(_self_instructions.memory[ref + i]),
        ) << (ofs * 8);
        value += tmp;
    }
    return value;
}

pub inline fn copy(dst: Ref, src: Ref, length: u8) void {
    for (0..length) |i|
        _self_instructions.memory[dst + i] = _self_instructions.memory[src + i];
    for (length..MACHINE_BYTES) |i|
        _self_instructions.memory[dst + i] = 0x00;
}

pub inline fn write(dst: Ref, value: ByteWidth, length: u8) void {
    const mask: ByteWidth = 0xff;
    for (0..length) |i| {
        const ofs: u5 = @intCast(i);
        const byte = shr(ByteWidth, value, ofs * 8) & mask;
        _self_instructions.memory[dst + i] = @as(u8, @intCast(byte));
    }
    for (length..MACHINE_BYTES) |i|
        _self_instructions.memory[dst + i] = 0x00;
}

pub inline fn deref(ref: Ref, length: u8) ByteWidth {
    return fetch(ref, length);
}
