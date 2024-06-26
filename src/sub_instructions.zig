const machine_config = @import("machine_config.zig");
const machine_ = @import("machine.zig");
const shr = @import("std").math.shr;

const ByteWidth = machine_config.ByteWidth;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;
const Machine: type = machine_.Machine;
const RegIdx: type = machine_.RegIdx;
const Cpu: type = machine_.Cpu;
const Ref: type = machine_config.Ref;
const Reg: type = machine_config.Reg;
const machine_bytes: u8 = @sizeOf(ByteWidth);

pub fn getReg(cpu: *Cpu, reg: u8) Reg {
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

pub fn fetch(mem_ref: [*]u8, src_bytes: u8) ByteWidth {
    var value: ByteWidth = 0;
    for (0..src_bytes) |i| {
        const ofs: u5 = @intCast(i);
        const tmp: ByteWidth = @as(u32, @intCast(mem_ref[i])) << (ofs * 8);
        value += tmp;
    }
    return value;
}

pub fn copy(dst: [*]u8, src: [*]u8, src_bytes: u8) void {
    for (0..src_bytes) |i|
        dst[i] = src[i];
    for (src_bytes..machine_bytes) |i|
        dst[i] = 0x00;
}

pub fn write(mem_ref: [*]u8, value: ByteWidth, src_bytes: u8) void {
    const mask: ByteWidth = 0xff;
    for (0..src_bytes) |i| {
        const ofs: u5 = @intCast(i);
        const byte = shr(ByteWidth, value, ofs * 8) & mask;
        mem_ref[i] = @as(u8, @intCast(byte));
    }
    for (src_bytes..machine_bytes) |i|
        mem_ref[i] = 0x00;
}
