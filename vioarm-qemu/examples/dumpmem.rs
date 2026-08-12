use vioarm_qemu::asm::{Asm, SP};
fn main() {
    let mut a = Asm::new();
    a.adds_imm8(0, 0);
    a.mov_imm32(4, 0x2000_0000);
    a.mov_imm32(0, 0x3344_1122);
    a.mov_imm32(1, 0xCDAB_EEFF);
    a.movw(2, 0x55AA);
    a.movw(3, 0x00FF);
    a.str_imm(0, 4, 0);
    a.strb_imm(1, 4, 4);
    a.strh_imm(0, 4, 6);
    a.str_wide(1, 4, 12);
    a.ls_reg(0, 0, 4, 3);
    a.ldr_imm(5, 4, 0);
    a.ldrb_imm(6, 4, 4);
    a.ldrh_imm(7, 4, 6);
    a.ls_reg(4, 6, 4, 3);
    a.ls_reg(7, 7, 4, 3);
    a.ldr_wide(8, 4, 12);
    a.ldrb_wide(8, 4, 4);
    a.ldrh_wide(9, 4, 6);
    a.ldrsb_wide(9, 4, 4);
    a.ldrsh_wide(9, 4, 6);
    a.ldr_lit(0, "pool");
    a.b("past_pool");
    a.align4();
    a.label("pool");
    a.dc_u32(0xDEAD_BEEF);
    a.label("past_pool");
    a.sub_sp(16);
    a.mov_reg(6, 13);
    a.add_high(7, 13);
    a.str_sp(0, 8);
    a.ldr_sp(6, 8);
    a.add_sp(16);
    a.push(0b0000_0001, false);
    a.push(0b0000_0010 | 0b1000_0000, true);
    a.pop(0b0000_0010 | 0b0100_0000, false);
    a.mov_imm32(5, 0x2000_0020);
    a.stmia(5, 0b0000_0011);
    a.ldmia(5, 0b0000_0011);
    a.stmia(5, 0b0001_0001);
    a.ldmia(5, 0b0001_0001);
    a.b("halt");
    a.label("halt");
    a.b("halt");
    let img = a.finish(SP).unwrap();
    for name in ["pool", "past_pool", "halt"] {
        println!("label {name} -> {:#x}", img.addr(name).unwrap());
    }
    let mut pc = 0x100usize;
    while pc < img.bytes.len() {
        let hi = u16::from_le_bytes([img.bytes[pc], img.bytes[pc + 1]]);
        let wide = hi >= 0xE800;
        if wide {
            let lo = u16::from_le_bytes([img.bytes[pc + 2], img.bytes[pc + 3]]);
            println!("{pc:04x}: {hi:04x} {lo:04x} (32)");
            pc += 4;
        } else {
            println!("{pc:04x}: {hi:04x} (16)");
            pc += 2;
        }
    }
}