pub const MEMORY_SIZE: ByteWidth = 0x1000;
pub const opr_sz: u8 = @sizeOf(ByteWidth);
pub const opc_sz: u8 = 1;

pub const ByteWidth: type = u32;

pub const Imm32: type = u32;
pub const Imm16: type = u16;
pub const Ref: type = ByteWidth;
pub const Reg: type = *ByteWidth;
