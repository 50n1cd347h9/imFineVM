const std = @import("std");
const machine_ = @import("./machine.zig");
const machine_config = @import("./machine_config.zig");
const sub_instructions = @import("./sub_instructions.zig");
const builtin = std.builtin;
const print = std.debug.print;
const pow = std.math.pow;
const write = sub_instructions.write;
const shr = std.math.shr;
const getReg = sub_instructions.getReg;
const fetch = sub_instructions.fetch;
const copy = sub_instructions.copy;

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

pub fn Instruction() type {
    return struct {
        const Self = @This();

        ip_ofs: u8,
        cpu: *Cpu,
        memory: [*]u8,
        ip: ByteWidth,
        sp: ByteWidth,
        ext: u8,
        len: u8,
        first: u8,
        imm_bytes: u8,
        first_reg: Reg,
        instruction: []*const fn (*Self) void,

        var func_vec: [@intFromEnum(InsCode.count)]*const fn (*Self) void = undefined;

        pub fn init(machine: *Machine) Self {
            const cpu = &machine.cpu;
            const ip = cpu.ip;
            const sp = cpu.sp;
            const memory = machine.memory;
            const ext = memory[ip] & ext_msk;
            const len = memory[ip + 1] >> 5;
            const first = memory[ip + 1] << 3 >> 5;
            const imm_bytes = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;
            const first_reg = getReg(cpu, first);
            initInstructions();

            return Self{
                .ip_ofs = 2,
                .cpu = cpu,
                .ip = ip,
                .sp = sp,
                .memory = memory,
                .ext = ext,
                .len = len,
                .first = first,
                .imm_bytes = imm_bytes,
                .first_reg = first_reg,
                .instruction = &func_vec,
            };
        }

        pub fn refresh(self: *Self) void {
            const cpu = self.cpu;
            const ip = cpu.ip;
            const sp = cpu.sp;
            const memory = self.memory;
            const ext = memory[ip] & ext_msk;
            const len = memory[ip + 1] >> 5;
            const first = memory[ip + 1] << 3 >> 5;
            const imm_bytes = if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;
            const first_reg = getReg(cpu, first);

            self.ip_ofs = 2;
            self.ip = ip;
            self.sp = sp;
            self.ext = ext;
            self.len = len;
            self.first = first;
            self.imm_bytes = imm_bytes;
            self.first_reg = first_reg;
        }

        fn initInstructions() void {
            func_vec[@intFromEnum(InsCode.add)] = add;
            //            func_vec[@intFromEnum(InsCode.sub)] = self.sub;
        }

        fn jump_switching(self: *Self) ByteWidth {
            var dst_loc: ByteWidth = 0;
            switch (self.ext) {
                Ext.imm => {
                    const dst_loc_rel: u32 = fetch(self.memory + self.ip + 2, self.imm_bytes);
                    if (dst_loc_rel >> 31 == 0b1) {
                        dst_loc = self.cpu.ip + opc_sz + 2 - ((dst_loc_rel << 1) >> 1);
                    } else {
                        dst_loc = self.cpu.ip + opc_sz + 2 + dst_loc_rel;
                    }
                },
                else => undefined,
            }
            return dst_loc;
        }

        fn arithmetic_switching(self: *Self, execute: fn (Reg, ByteWidth) ByteWidth) ByteWidth {
            const memory: [*]u8 = self.memory;
            const ip: ByteWidth = self.ip;
            const imm_bytes: u8 = self.imm_bytes;
            const dst_reg: Reg = self.first_reg;
            const cpu: *Cpu = self.cpu;
            self.ip_ofs += self.imm_bytes;
            switch (self.ext) {
                Ext.imm => {
                    const src = fetch(memory + ip + 2, imm_bytes);
                    return execute(dst_reg, src);
                },
                Ext.reg => {
                    const src_reg: Reg = getReg(cpu, memory[ip + 2]);
                    return execute(dst_reg, src_reg.*);
                },
                Ext.imm_ref => {
                    const src_ref: Ref = fetch(memory + ip + 2, imm_bytes);
                    const src = fetch(memory + src_ref, imm_bytes);
                    return execute(dst_reg, src);
                },
                Ext.reg_ref => {
                    const src_ref: Ref = (getReg(cpu, memory[ip + 2])).*;
                    const src = fetch(memory + src_ref, machine_bytes);
                    return execute(dst_reg, src);
                },
                else => unreachable,
            }
        }

        fn pop(self: *Self) void {
            self.ip_ofs += self.imm_bytes;
            defer {
                self.cpu.ip += self.ip_ofs;
                self.cpu.sp += machine_bytes;
            }
            switch (self.ext) {
                Ext.reg => {
                    const dst_reg: Reg = getReg(self.cpu, self.memory[self.ip + 2]);
                    const src = fetch(self.memory + self.sp, machine_bytes);
                    dst_reg.* = src;
                },
                Ext.imm_ref => {
                    const dst_ref: Ref = fetch(self.memory + self.ip + 2, self.imm_bytes);
                    copy(self.memory + dst_ref, self.memory + self.sp, machine_bytes);
                },
                Ext.reg_ref => {
                    const dst_ref = (getReg(self.cpu, self.memory[self.ip + 2])).*;
                    copy(self.memory + dst_ref, self.memory + self.sp, machine_bytes);
                },
                else => {
                    return;
                },
            }
        }

        fn push(self: *Self) void {
            self.ip_ofs += self.imm_bytes;
            defer {
                self.cpu.ip += self.ip_ofs;
                self.cpu.sp -= machine_bytes;
            }
            switch (self.ext) {
                Ext.imm => {
                    copy(self.memory + self.sp - machine_bytes, self.memory + self.ip + 2, self.imm_bytes);
                },
                Ext.reg => {
                    const src = (getReg(self.cpu, self.memory[self.ip + 2])).*;
                    write(self.memory + self.sp - machine_bytes, src, machine_bytes);
                },
                Ext.imm_ref => {
                    const src_ref: Ref = fetch(self.memory + self.ip + 2, machine_bytes);
                    copy(self.memory + self.sp - machine_bytes, self.memory + src_ref, machine_bytes);
                },
                Ext.reg_ref => {
                    const src_ref: Ref = (getReg(self.cpu, self.memory[self.ip + 2])).*;
                    copy(self.memory + self.sp - machine_bytes, self.memory + src_ref, machine_bytes);
                },
                else => {
                    return;
                },
            }
        }

        pub const hoge = struct {
            fn lambda(self: *Self) void {
                print("ip_ofs = {d}\n", .{self.ip_ofs});
            }
        }.lambda;

        pub const add = struct {
            fn lambda(self: *Self) void {
                defer {
                    self.cpu.ip += self.ip_ofs;
                    self.refresh();
                }
                _ = self.arithmetic_switching(struct {
                    pub const _self = self;
                    fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
                        dst_reg.* += src;
                        return 0;
                    }
                }.execute);
            }
        }.lambda;

        fn sub(self: *Self) void {
            defer {
                self.cpu.ip += self.ip_ofs;
            }
            _ = self.arithmetic_switching(struct {
                fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
                    dst_reg.* -= src;
                    return 0;
                }
            }.execute);
        }

        fn mul(self: *Self) void {
            defer {
                self.cpu.ip += self.ip_ofs;
            }
            _ = self.arithmetic_switching(struct {
                fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
                    dst_reg.* *= src;
                    return 0;
                }
            }.execute);
        }

        fn div(self: *Self) void {
            defer {
                self.cpu.ip += self.ip_ofs;
            }
            _ = self.arithmetic_switching(struct {
                fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
                    dst_reg.* /= src;
                    return 0;
                }
            }.execute);
        }

        fn and_(self: *Self) void {
            defer {
                self.cpu.ip += self.ip_ofs;
            }
            _ = self.arithmetic_switching(struct {
                fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
                    dst_reg.* &= src;
                    return 0;
                }
            }.execute);
        }

        fn or_(self: *Self) void {
            defer {
                self.cpu.ip += self.ip_ofs;
            }
            _ = self.arithmetic_switching(struct {
                fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
                    dst_reg.* |= src;
                    return 0;
                }
            }.execute);
        }

        fn xor(self: *Self) void {
            defer {
                self.cpu.ip += self.ip_ofs;
            }
            _ = self.arithmetic_switching(struct {
                fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
                    dst_reg.* ^= src;
                    return 0;
                }
            }.execute);
        }

        fn shl_(self: *Self) void {
            defer {
                self.cpu.ip += self.ip_ofs;
            }
            _ = self.arithmetic_switching(struct {
                fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
                    dst_reg.* = dst_reg.* << @as(u5, @intCast(src));
                    return 0;
                }
            }.execute);
        }

        fn cmp(self: *Self) void {
            var flag: ByteWidth = 0b00;
            defer {
                self.cpu.ip += self.ip_ofs;
                self.cpu.flag &= @as(u8, @intCast(flag));
            }
            flag = self.arithmetic_switching(struct {
                fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
                    const dst: ByteWidth = dst_reg.*;
                    if (dst > src) {
                        return 0b00;
                    } else if (dst == src) {
                        return 0b01;
                    } else {
                        return 0b10;
                    }
                }
            }.execute);
        }

        fn ld(self: *Self) void {
            if (self.memory[self.ip] >> 2 == @intFromEnum(InsCode.ldr)) {
                self.ldr();
            } else {
                self.ldm();
            }
        }

        // load to register
        fn ldr(self: *Self) void {
            self.ip_ofs += self.imm_bytes;
            defer {
                self.cpu.ip += self.ip_ofs;
            }
            const memory: [*]u8 = self.memory;
            const ip: ByteWidth = self.ip;
            const imm_bytes: u8 = self.imm_bytes;
            const dst_reg: Reg = self.first_reg;
            const cpu: *Cpu = self.cpu;
            switch (self.ext) {
                Ext.imm => {
                    const src = fetch(memory + ip + 2, imm_bytes);
                    dst_reg.* = src;
                },
                Ext.reg => {
                    const src_reg: Reg = getReg(cpu, memory[ip + 2]);
                    dst_reg.* = src_reg.*;
                },
                Ext.imm_ref => {
                    const src_ref: Ref = fetch(memory + ip + 2, imm_bytes);
                    const src = fetch(memory + src_ref, machine_bytes);
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
        fn ldm(self: *Self) void {
            self.ip_ofs += self.imm_bytes;
            defer {
                self.cpu.ip += self.ip_ofs;
            }
            const memory: [*]u8 = self.memory;
            const ip: ByteWidth = self.ip;
            const imm_bytes: u8 = self.imm_bytes;
            const dst_reg: Reg = self.first_reg;
            const dst_ref: Ref = dst_reg.*;
            const cpu: *Cpu = self.cpu;
            switch (self.ext) {
                Ext.imm => {
                    copy(memory + dst_ref, memory + ip + 2, imm_bytes);
                },
                Ext.reg => {
                    const src_reg: Reg = getReg(cpu, memory[ip + 2]);
                    write(memory + dst_ref, src_reg.*, machine_bytes);
                },
                Ext.imm_ref => {
                    const src_ref: Ref = fetch(memory + ip + 2, imm_bytes);
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

        fn jmp(self: *Self) void {
            var dst_loc: ByteWidth = 0;
            defer {
                self.cpu.ip = dst_loc;
            }
            dst_loc = self.jump_switching();
        }

        fn jg(self: *Self) void {
            var dst_loc: ByteWidth = 0;
            defer {
                self.cpu.ip = dst_loc;
            }
            if ((self.cpu.flag & 0b00) == 0b00) {
                dst_loc = self.jump_switching();
            } else {
                dst_loc += self.cpu.ip + opc_sz;
            }
        }

        fn jz(self: *Self) void {
            var dst_loc: ByteWidth = 0;
            defer {
                self.cpu.ip = dst_loc;
            }
            if ((self.cpu.flag & 0b01) == 0b01) {
                dst_loc = self.jump_switching();
            } else {
                dst_loc += self.cpu.ip + opc_sz;
            }
        }

        fn jl(self: *Self) void {
            var dst_loc: ByteWidth = 0;
            defer {
                self.cpu.ip = dst_loc;
            }
            if ((self.cpu.flag & 0b10) == 0b10) {
                dst_loc = self.jump_switching();
            } else {
                dst_loc += self.cpu.ip + opc_sz;
            }
        }

        fn call(self: *Self) void {
            defer {
                self.cpu.sp -= machine_bytes;
            }
            const ret_addr = self.ip + self.ip_ofs + self.imm_bytes;
            write(self.memory + self.sp - machine_bytes, ret_addr, machine_bytes);
            self.jmp();
        }

        fn ret(self: *Self) void {
            const ret_addr: ByteWidth = fetch(self.memory + self.sp, machine_bytes);
            defer {
                self.cpu.sp += machine_bytes;
                self.cpu.ip = ret_addr;
            }
        }
    };
}

fn nop(machine: *Machine) void {
    const ip_ofs: u8 = opc_sz;
    const cpu: *Cpu = &machine.cpu;
    defer {
        cpu.ip += ip_ofs;
    }
}
