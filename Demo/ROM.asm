const zero     0x00 ; Location of 0x00 value
const unity    0x01 ; Location of 0x01 value
const maxbyte  0x02 ; Location of 0xFF value

const reg_B    0x10 ; Location of Register B
const reg_C    0x11 ; Location of Register C
const reg_D    0x12 ; Location of Register D
const reg_E    0x13 ; Location of Register E

const mmu_stc  0x20 ; Stimulus Current
const cntr     0x21 ; Counter variable


main:

push A            ; Save A.
ld A,(cntr)       ; Load counter,
add A,(unity)     ; increment,
ld (cntr),A       ; and store.
rr A              ; Shift right
rr A              ; a total
rr A              ; of four 
rr A              ; times.
ld (mmu_stc),A    ; Set stimulus current.
pop A             ; Restore A.

ld IX,zero
ld A,(IX)
ld (0x22),A
inc IX


ld A,(zero)
delay_loop:
add A,(unity)
jp z,delay_loop
jp main
