const std = @import("std");
const builtin = std.builtin;

pub const MEMORY_SIZE: ByteWidth = 0x10000;
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

pub const RegIdx = enum(usize) { ip, sp, fp, flag, gr0, gr1, count };
pub const InsCode = enum(usize) { push, pop, add, sub, mul, div, and_, or_, xor, shl_, ldr, ldm, cmp, jmp, jg, jz, jl, call, ret, nop, count };
pub const Ext = struct {
    pub const imm = 0b00;
    pub const reg = 0b01;
    pub const imm_ref = 0b10;
    pub const reg_ref = 0b11;
};
