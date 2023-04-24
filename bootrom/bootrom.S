.section .initial_jump
.global _start

.align 4
_start:
    li sp, 0x83800000
    call main

    //li x1, 0x00000000
    //li x2, 0x00000000
    //li x3, 0x00000000
    //li x4, 0x00000000
    //li x5, 0x00000000
    //li x6, 0x00000000
    //li x7, 0x00000000
    //li x8, 0x00000000
    //li x9, 0x00000000
    //li x10, 0x00000000
    //li x11, 0x00000000
    //li x12, 0x00000000
    //li x13, 0x00000000
    //li x14, 0x00000000
    //li x15, 0x00000000
    //li x16, 0x00000000
    //li x17, 0x00000000
    //li x18, 0x00000000
    //li x19, 0x00000000
    //li x20, 0x00000000
    //li x21, 0x00000000
    //li x22, 0x00000000
    //li x23, 0x00000000
    //li x24, 0x00000000
    //li x25, 0x00000000
    //li x26, 0x00000000
    //li x27, 0x00000000
    //li x28, 0x00000000
    //li x29, 0x00000000
    //li x30, 0x00000000
    //li x31, 0x00000000

    //li x1, 0x80000000
    //csrw tselect, x1
    //li x1, 0x00000000
    //csrw tdata1, x1

    li x1, 0x80000000
    li x11, 0x83fffa00
    jalr x0, x1, 0

//_stop:
//    j _stop
