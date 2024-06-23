const std = @import("std");
const builtin = std.builtin;

pub const MEMORY_SIZE: ByteWidth = 0x5000;
pub const opr_sz: u8 = @sizeOf(ByteWidth);
pub const opc_sz: u8 = 1;

pub const ByteWidth: type = u32;
pub const SignedByteWidth: type = @Type(.{ .Int = .{
    .bits = @typeInfo(ByteWidth).Int.bits,
    .signedness = builtin.Signedness.signed,
} });

pub const Imm32: type = u32;
pub const Imm16: type = u16;
pub const Imm8: type = u8;
pub const Ref: type = ByteWidth;
pub const Reg: type = *ByteWidth;
