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
pub const InsCode = enum(usize) { push, pop, add, sub, mul, div, and_, or_, xor, shl_, ldr, ldm, cmp, jmp, jg, jz, jl, nop, count };
pub const Ext = enum(u2) { imm, reg, imm_ref, reg_ref };
pub const Len = enum(u3) { _0, _8, _16, _32, _64 };
// pub const Len: [8]?type = { null, u8, u16, u32, u64};
