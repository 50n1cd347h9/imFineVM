const std = @import("std");
const builtin = std.builtin;

pub const ByteWidth: type = u32;
pub const SignedByteWidth: type = @Type(.{ .int = .{
    .bits = @typeInfo(ByteWidth).int.bits,
    .signedness = builtin.Signedness.signed,
} });

pub const MEMORY_SIZE: ByteWidth = 0x10000;
pub const EXT_MSK: u8 = 0b00000011;
pub const MACHINE_BYTES: u8 = @sizeOf(ByteWidth);
pub const OPR_SZ: u8 = @sizeOf(ByteWidth);
pub const OPC_SZ: u8 = 1; // this must not be changed

pub const Imm32: type = u32;
pub const Imm16: type = u16;
pub const Imm8: type = u8;
pub const Ref: type = ByteWidth;
pub const Reg: type = *ByteWidth;

pub const RegIdx = enum(usize) { ip, sp, fp, flag, gr0, gr1, count };
pub const InsCode = enum(usize) {
    push,
    pop,
    add,
    sub,
    mul,
    div,
    _and,
    _or,
    xor,
    _shl,
    ldr,
    ldm,
    cmp,
    jmp,
    jg,
    jz,
    jl,
    call,
    ret,
    nop,
    count,
};
pub const Ext = struct {
    pub const imm = 0b00;
    pub const reg = 0b01;
    pub const imm_ref = 0b10;
    pub const reg_ref = 0b11;
};
// pub const Ext = enum(u2) {
//     imm = 0b00,
//     reg = 0b01,
//     imm_ref = 0b10,
//     reg_ref = 0b11,
// };
