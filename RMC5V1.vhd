-- RISC Microcontroller, Five-Bit Opcodes (RMC5) 
--
-- Copyright (C) 2026 Kevan Hashemi, Open Source Instruments Inc.
--
-- This program is free software; you can redistribute it and/or
-- modify it under the terms of the GNU General Public License
-- as published by the Free Software Foundation; either version 2
-- of the License, or (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program; if not, write to the Free Software
-- Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, 
-- USA.

-- V1.1, [29-OCT-26] Combinatorial instruction memory interface,
-- combinatorial read interface with external process memory. The 
-- Write Data Strobe (WDS) signal marks when the data provided -- by the microcontroller should be written to the process memory
-- location specified by the mcu address. The WDS signal will be
-- asserted some combinatorial delay after the rising edge of 
-- CK on an instruction that writes to process memory. It will
-- persist until after the next CK rising edge.

library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- The microcontroller requires seperate instruction and process memories. 
-- Both memories have maximum size 128 locations. The instruction
-- memory locations are 12 bits long: 5 for opcode and 7 for an
-- operand. The operand can be an address in either memories, or it can be
-- ignored. We might use the operand for other purposes in future versions
-- of the microcontroller, but for now we are trying to keep it small.

-- The instruction memory and process memory are external to the MCU entity.
-- The instruction memory should be read-only memory (ROM) with no clock, so
-- that it is implemented as combinatorial logic. The program counter connects 
-- directly to the ROM and the instruction appears in response with no clock 
-- delays. The process memory should be combinatorial for read cycles and
-- perform a write to the mcu address location when it sees WDS asserted on
-- the rising edge of its write clock. The WDS signal also tells the external
-- logic not drive a shared data bus, if one exist. The write can take place
-- wither on the falling edge of CK, half a CK period after the instruction
-- processing begins, or on the rising edge of CK, a whole CK period after 
-- processing begins.

-- The IRQ input is the interrupt request line from external logic. The memory 
-- management unit (MMU) must provide a way for the MCU to clear the IRQ signal 
-- so it can get back to its main program. The IRSV output is a flag to show
-- that an interrupt is being serviced. The flag is set on the falling edge of
-- the clock, which is convenient for the clock boost state machines we tend
-- to use in our battery-powered devices.

-- The RESET signal will put the program counter back to start_pc and the stack
-- pointer to zero. The stack is upward-going. The processor expects RAM for
-- the stack in the locations zero and up, but we can set the stack pointer to
-- some other value with an instruction. Interrupts execute from interrupt_pc.

entity RMC5_MCU is 
	generic (
		addr_len : integer := 7;
		data_len : integer := 8;
		start_pc : integer := 0;
		interrupt_pc : integer := 3;
		start_sp : integer := 0;
		opcode_len : integer := 5;
		operand_len : integer := addr_len;
		instr_len : integer := opcode_len+operand_len
	);
	port (
		prog_cntr : out std_logic_vector(addr_len-1 downto 0); -- Outgoing Program Counter
		prog_data : in std_logic_vector(instr_len-1 downto 0); -- Incoming Program Data
		mcu_data_out : out std_logic_vector(data_len-1 downto 0); -- Outgoing Data Bus
		mcu_data_in : in std_logic_vector(data_len-1 downto 0); -- Incoming Data Bus
		mcu_addr : out std_logic_vector(addr_len-1 downto 0); -- Outgoing Address Bus
		WDS : out boolean; -- Write Data Strobe
		IRQ : in boolean; -- Interrupt Request
		ISRV : out boolean; -- Interrupt Service
		RESET : in std_logic; -- Hard Reset
		CK : in std_logic); -- The clock, duty cycle 50%.
end;

architecture behavior of RMC5_MCU is 

-- Instruction types.
	subtype instruction_type is std_logic_vector(instr_len-1 downto 0);
	subtype opcode_type is std_logic_vector(opcode_len-1 downto 0);
	subtype operand_type is std_logic_vector(addr_len-1 downto 0);
	subtype data_type is std_logic_vector(data_len-1 downto 0);

-- Definition of operation codes. We have the constant name we will be using
-- in the VHDL definition of the MCU to refer to an operation, followed by 
-- the hexadecimal byte value by which this operation will be invoked as a 
-- program is being executed, and the instruction pneumonic that shows what
-- the instruction does, and how it is to be written in the RMC5 assembly 
-- language. We use "(n)" for the contents of process memory address "n",
-- "jp nz" for jump if not zero, "ld" for "load", and so on. Our syntax for 
-- assembler code follows that of the venerable Z80 microprocessor.

	-- Program flow: 00-07
	constant nop      : opcode_type := "00000"; -- 00 nop
	constant jp_n     : opcode_type := "00001"; -- 01 jp n
	constant jp_nz_n  : opcode_type := "00010"; -- 02 jp nz,n
	constant jp_z_n   : opcode_type := "00011"; -- 03 jp z,n
	constant jp_nc_n  : opcode_type := "00100"; -- 04 jp nc,n
	constant jp_c_n   : opcode_type := "00101"; -- 05 jp c,n
	constant call_n   : opcode_type := "00110"; -- 06 call n
	constant ret_cll  : opcode_type := "00111"; -- 07 ret

	-- Stack, interrupt return, and memory transfer: 08-0E
	constant ret_int  : opcode_type := "01000"; -- 08 rti
	constant push_A   : opcode_type := "01001"; -- 09 push A
	constant pop_A    : opcode_type := "01010"; -- 0A pop A
	constant push_F   : opcode_type := "01011"; -- 0B push F
	constant pop_F    : opcode_type := "01100"; -- 0C pop F
	constant ld_m_A   : opcode_type := "01101"; -- 0D ld (n),A
	constant ld_A_m   : opcode_type := "01110"; -- 0E ld A,(n)

	-- Arithmetic: 10-13
	constant add_A_m  : opcode_type := "10000"; -- 10 add A,(n)
	constant adc_A_m  : opcode_type := "10001"; -- 11 adc A,(n)
	constant sub_A_m  : opcode_type := "10010"; -- 12 sub A,(n)
	constant sbc_A_m  : opcode_type := "10011"; -- 13 sbc A,(n)

	-- Logic: 14-16
	constant and_A_m  : opcode_type := "10100"; -- 14 and A,(n)
	constant or_A_m   : opcode_type := "10101"; -- 15 or A,(n)
	constant xor_A_m  : opcode_type := "10110"; -- 16 xor A,(n)

	-- Rotations and flag control: 18-1C
	constant rl_A     : opcode_type := "11000"; -- 18 rl A
	constant rr_A     : opcode_type := "11001"; -- 19 rr A
	constant clr_aflg : opcode_type := "11010"; -- 1A clrf
	constant clr_iflg : opcode_type := "11011"; -- 1B clri
	constant set_iflg : opcode_type := "11100"; -- 1C seti
	
-- Attributes to guide the compiler.
	attribute syn_keep : boolean;
	attribute nomerge : string;

-- Arithmetic Logic Unit inputs.
	signal alu_in_x, alu_in_y : data_type;
	signal alu_cin : boolean;
	subtype alu_ctrl_type is unsigned(3 downto 0);
	signal alu_ctrl : alu_ctrl_type;

-- Arithmetic Logic Unit outputs. We force the compiler to keep the 
-- main alu output signal. Doing so reduces the code size.
	signal alu_out : data_type;
	signal alu_cout : boolean;
	attribute syn_keep of alu_out : signal is true;
	attribute nomerge of alu_out : signal is "";	

-- Arithmetic Logic Unit control codes. Not all of these need be supported by
-- the MCU instruction set. Any that are not used will be optimized away by
-- the compiler.
	constant alu_cmd_add  : alu_ctrl_type := "0000"; -- Add X and Y
	constant alu_cmd_sub  : alu_ctrl_type := "0001"; -- Subtract Y from X
	constant alu_cmd_and  : alu_ctrl_type := "0100"; -- Bitwise AND of X and Y
	constant alu_cmd_xor  : alu_ctrl_type := "0101"; -- Bitwise XOR of X and Y
	constant alu_cmd_or   : alu_ctrl_type := "0110"; -- Bitwise OR of X and Y
	constant alu_cmd_rl   : alu_ctrl_type := "1000"; -- Rotate Left of X
	constant alu_cmd_rlc  : alu_ctrl_type := "1001"; -- Rotate Left Circuiar of X
	constant alu_cmd_sla  : alu_ctrl_type := "1010"; -- Shift Left Arithmetic of X
	constant alu_cmd_rr   : alu_ctrl_type := "1100"; -- Rotate Right of X
	constant alu_cmd_rrc  : alu_ctrl_type := "1101"; -- Rotate Right Circular of X
	constant alu_cmd_sra  : alu_ctrl_type := "1110"; -- Shift Right Arithmetic of X
	constant alu_cmd_srl  : alu_ctrl_type := "1111"; -- Shift Right Logical of X

-- Instruction fields.
	signal opcode : opcode_type;
	signal operand : operand_type;

-- MCU Registers. The Accumulator, or Register A, in which we get the result 
-- of eight-bit arithmetic operations, logical operations, and shifts and 
-- rotations. When such an operation requires a second operand, we can use 
-- either Register B or a one-byte constant.
	signal reg_A : data_type;

-- The flags are bits in the Flag Register, but we work with them as separate
-- boolean signals so as to simplify our code. We have the Zero flag, Carry
-- flag, Sign flag, and Interrupt flag.
	signal flag_Z, flag_C, flag_I : boolean;

-- The Stack Pointer (SP) we use to manage an upward-growing stack. The
-- Stack Pointer points to the empty location on top of the stack, which 
-- is the byte above the one most recently pushed onto the stack. When we 
-- push a byte onto the stack, we write to the stack and then increment
-- the stack pointer. When we pop from the stack, we decrement the stack
-- pointer, then read from the stack. 
	signal reg_SP : operand_type;
	
-- This signal is the one the MCU sets on the rising edge of CK to
-- indicated that it is executing an interrupt routine. We will delay
-- until the falling edge of CK to make ISRV.
	signal RISRV : boolean;
	
-- The state values.
	signal state : integer range 0 to 2;
	constant read_opcode : integer := 0;
	constant read_stack : integer := 1;
	constant incr_pc : integer := 2;
	
-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;

begin 

-- Global opcode and operand variables separate the top and bottom sections
-- of the instruction that is currently output from the instruction memory.
	opcode <= prog_data(instr_len-1 downto instr_len-opcode_len);
	operand <= prog_data(addr_len-1 downto 0);

-- The Arithmetic Logic Unit provides an eight-bit adder-subtractor with 
-- carry in and carry out, as well as logical operations AND, OR, and XOR.
	ALU : process (all) is
	variable result : unsigned(data_len downto 0);
	variable inx, iny : unsigned(data_len-1 downto 0); 
	begin 
		inx := unsigned(alu_in_x);
		iny := unsigned(alu_in_y);
		result := ('0' & inx) + ('0' & iny);
		
		case alu_ctrl is
		
		-- Add X and Y with Carry
		when alu_cmd_add =>
			if alu_cin then 
				result := ('0' & inx) + ('0' & iny) + "000000001";
			else 
				result := ('0' & inx) + ('0' & iny); 
			end if;
			
		-- Subtract Y from X with Borrow
		when alu_cmd_sub =>
			if alu_cin then 
				result := ('0' & inx) - ('0' & iny) - "000000001";
			else 
				result := ('0' & inx) - ('0' & iny); 
			end if;
			
		-- Bit-Wise Exclusive OR of A and B
		when alu_cmd_xor =>
			result := '0' & (inx xor iny);
			
		-- Bit-Wise OR of A and B
		when alu_cmd_or =>
			result := '0' & (inx or iny);
					
		-- Bit-Wise Logical AND of X and Y
		when alu_cmd_and =>
			result := '0' & (inx and iny);
					
		-- Rotate Left of X.
		when alu_cmd_rl  => 
			result(data_len downto 1) := inx(data_len-1 downto 0);
			result(0) := to_std_logic(alu_cin);	
			
		 -- Rotate Left Circuiar of X
		when alu_cmd_rlc => 
			result(data_len-1 downto 1) := inx(data_len-2 downto 0);
			result(0) := inx(data_len-1);	
			result(data_len) := inx(data_len-1);
			
		-- Rotate Right of X
		when alu_cmd_rr  =>
			result(data_len-1) := to_std_logic(alu_cin);
			result(data_len-2 downto 0) := inx(data_len-1 downto 1);
			result(data_len) := inx(0);		
			
		-- Rotate Right Circular of X
		when alu_cmd_rrc =>  
			result(data_len-2 downto 0) := inx(data_len-1 downto 1);
			result(data_len-1) := inx(0);		
			result(data_len) := inx(0);		
			
		-- Shift Left Arithmetic of X
		when alu_cmd_sla =>  
			result(data_len downto 1) := inx(data_len-1 downto 0);
			result(0) :='0';	
			
		-- Shift Right Arithmetic of X
		when alu_cmd_sra =>  
			result(data_len-2 downto 0) := inx(data_len-1 downto 1);
			result(data_len-1) := inx(data_len-1);		
			result(data_len) := inx(0);		
			
		-- Shift Right Logical of X
		when alu_cmd_srl => 
			result(data_len-2 downto 0) := inx(data_len-1 downto 1);
			result(data_len-1) := '0';		
			result(data_len) := inx(0);		

		-- Let the default values stand.
		when others =>
			null;
			
		end case;
		
		alu_out <= std_logic_vector(result(data_len-1 downto 0));
		alu_cout <= result(data_len) = '1';
	end process;

-- The Central Processing Unit reads and writes from instruction, process, and 
-- stack memory.
	MCU : process (CK, RESET) is
	variable next_A : data_type;
	variable next_flag_Z, next_flag_C, next_flag_I : boolean;
	variable next_pc, next_SP : operand_type;
	variable next_RISRV : boolean;
	variable next_state : integer range 0 to 3;
						
	begin

		-- Reset the cpu state and program counter until we enter standby mode.
		if (RESET ='1') then 
			state <= read_opcode;
			prog_cntr <= std_logic_vector(to_unsigned(start_pc,addr_len)); 
			reg_SP <= std_logic_vector(to_unsigned(start_sp,addr_len)); 
			reg_A <= (others => '0');
			flag_Z <= false;
			flag_C <= false;
			flag_I <= false;
			RISRV <= false;
			
		-- Otherwise we repond to the rising edge of CK.
		elsif rising_edge(CK) then
		
			-- Define default next values. We end up stating explicitly what
			-- the next state and next program counter will be.
			next_state := read_opcode;
			next_pc := std_logic_vector(unsigned(prog_cntr)+1);
			next_A := reg_A;
			next_SP := reg_SP;
			next_flag_C := flag_C;
			next_flag_Z := flag_Z;
			next_flag_I := flag_I;
			next_RISRV := RISRV;
			
			-- Read the first byte of the instruction. Most instructions will be executed
			-- in one state, but a few need more states.
			if (state = read_opcode) then
					
				-- We handle an interrupt pushing the program counter onto the 
				-- stack and at the same time loading the program counter with
				-- the interrupt vector. We set the I flag to disable further
				-- interrupts. The interrupt routine must do something to clear 
				-- the Interrupt Request signal (IRQ) or else the main process 
				-- will be interrupted again upon return. The interrupt routine 
				-- should use the return from interrupt instruction (ret_int or 
				-- "rti"), which will clear flag_I upon return. Clock Cycles = 1.
				if IRQ and (not flag_I) then
					next_SP := std_logic_vector(unsigned(reg_SP)+1);
					next_flag_I := true;
					next_state := read_opcode;
					next_pc := std_logic_vector(to_unsigned(interrupt_pc,addr_len));
					next_RISRV := true;
					
				-- This is not an interrupt, so we are going to decode the 
				-- instruction, which is made up of an opcode and an operand.
				else
					case opcode is
					
					-- The no-operation instruction. Clock Cycles = 1.
					when nop => 
						next_state := read_opcode;
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);
						
					-- Return from interrupt, we start by decremending the stack pointer 
					-- and freezing the program counter. Clock Cycles = 2.
					when ret_int =>
						next_SP := std_logic_vector(unsigned(reg_SP)-1);
						next_state := read_stack;
						next_pc := prog_cntr;
				
					-- Indirect eight-bit read operations using operand as address. Clock 
					-- Cycles = 1.
					when ld_A_m =>
						next_A := mcu_data_in;
						next_state := read_opcode;
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);
						
					-- Indirect eight-bit write operations using operand as address. Clock 
					-- Cycles = 1.
					when ld_m_A =>
						next_state := read_opcode;
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);

					-- Push operations for A and F onto the eight-bit wide stack. Clock
					-- Cycles = 1.
					when push_A | push_F => 
						next_SP := std_logic_vector(unsigned(reg_SP)+1);
						next_state := read_opcode;
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					
					-- Pop instructions for eight-bit registers. Clock Cycles = 2.
					when pop_A | pop_F =>
						next_SP := std_logic_vector(unsigned(reg_SP)-1);
						next_state := read_stack;	
						next_pc := prog_cntr;
						
					-- Add and subtract. Set Z and C flags. Clock Cycles = 1. 
					when add_A_m | sub_A_m | adc_A_m | sbc_A_m =>
						next_A := alu_out;
						next_flag_Z := (unsigned(alu_out) = 0);
						next_flag_C := alu_cout;
						next_state := read_opcode;
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);

					-- Eight-bit logical AND. Sets the Z flag only. Clock Cycles = 1.
					when and_A_m  => 
						next_A := alu_out;
						next_flag_Z := (unsigned(alu_out) = 0);
						next_state := read_opcode;
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);

					-- Eight-bit logical OR. Sets the Z flag only. Clock Cycles = 1.
					when or_A_m  => 
						next_A := alu_out;
						next_flag_Z := (unsigned(alu_out) = 0);
						next_state := read_opcode;
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);

					-- Eight-bit logical XOR. Sets the Z flag only. Clock Cycles = 1.
					when xor_A_m  => 
						next_A := alu_out;
						next_flag_Z := (unsigned(alu_out) = 0);
						next_state := read_opcode;
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);

					-- Rotate left. Set C flag. Clock Cycles = 1.
					when rl_A =>
						next_A := alu_out;
						next_flag_C := alu_cout;
						next_state := read_opcode;
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);
						
					-- Rotate right. Set C flag. Clock Cycles = 1.
					when rr_A =>
						next_A := alu_out;
						next_flag_C := alu_cout;
						next_state := read_opcode;
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);
						
					-- Clear arithmetic flags: a single instruction clears the zero (Z), 
					-- and carry (C) flags, but not the interrupt (I) flag;
					when clr_aflg =>
						next_flag_Z := false;
						next_flag_C := false;
						next_state := read_opcode;
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);

					-- Clear interrupt flag, so as to enable interrupts.
					when clr_iflg =>
						next_flag_I := false;
						next_state := read_opcode;
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);

					-- Set interrupt flag, so as to disable interrupts.
					when set_iflg =>
						next_flag_I := true;
						next_state := read_opcode;
						next_pc := std_logic_vector(unsigned(prog_cntr)+1);
						
					-- Call a subroutine. We push the program counter onto the stack and
					-- jump to the instruction memory location given by the operand. Clock 
					-- Cycles = 1.
					when call_n =>
						next_SP := std_logic_vector(unsigned(reg_SP)+1);
						next_state := read_opcode;
						next_pc := operand;

					-- Return from subroutine, we start by decremending the stack
					-- pointer and freezing the program counter. After that we 
					-- will restore the program counter from the stack, and then
					-- increment the program counter. Clock Cycles = 3.
					when ret_cll =>
						next_SP := std_logic_vector(unsigned(reg_SP)-1);
						next_state := read_stack;
						next_pc := prog_cntr;

					-- Decide if we should jump to the address specified by the operands. If
					-- so, we will jump immediately by loading the program counter with a new 
					-- value. Clock Cycles = 1.
					when jp_n | jp_z_n | jp_nz_n | jp_c_n | jp_nc_n=>
						if (opcode = jp_n) or 
							((opcode = jp_z_n) and flag_Z) or
							((opcode = jp_nz_n) and (not flag_Z)) or
							((opcode = jp_c_n) and flag_C) or
							((opcode = jp_nc_n) and (not flag_C)) then
							next_pc := operand;
						else 
							next_pc := std_logic_vector(unsigned(prog_cntr)+1);
						end if;
						next_state := read_opcode;
						
					-- Let the defaults stand;
					when others => null;					
						
					end case;
				end if;
				
			-- Read the value pointed to by the stack pointer and do what our opcode
			-- tells us we should do with it.
			elsif (state = read_stack) then			
				case opcode is
				
				-- For the pop instructions, we take the stack value and load into a
				-- flag or registesr.
				when pop_A | pop_F =>
					case opcode is
						when pop_A => next_A := mcu_data_in;
						when pop_F => 
							next_flag_I := (mcu_data_in(7) = '1');
							next_flag_C := (mcu_data_in(2) = '1');		
							next_flag_Z := (mcu_data_in(0) = '1');
						when others => null;
					end case;
					next_pc := std_logic_vector(unsigned(prog_cntr)+1);
					next_state := read_opcode;
					
				-- Read the program counter from the stack. After this, we are going
				-- to increment the counter.
				when ret_cll =>
					next_pc := mcu_data_in(addr_len-1 downto 0);
					next_state := incr_pc;	
					
				-- Read the program counter from the stack and continue execution.
				when ret_int =>
					next_pc := mcu_data_in(addr_len-1 downto 0);
					next_flag_I := false;
					next_RISRV := false;
					next_state := read_opcode;
					
				-- Let the defaults stand;
				when others => null;

				end case;
							
			-- The increment-program-counter state increments the program counter and moves
			-- to the read-opcode state. We use it when returning from a subroutine or a
			-- software interrupt, to increment the program counter we popped off the stack
			-- so that it points to the next instruction after the call or software interrupt.
			elsif (state = incr_pc) then
				next_state := read_opcode;
				next_pc := std_logic_vector(unsigned(prog_cntr)+1);	

			end if;
			
			-- Update Z after arithmetic and logical operations.
			if (state = read_opcode) and
			   ((opcode = add_A_m) or
				(opcode = adc_A_m) or
				(opcode = sub_A_m) or
				(opcode = sbc_A_m) or
				(opcode = and_A_m) or
				(opcode = or_A_m) or
				(opcode = xor_A_m)) and
			   not (IRQ and (not flag_I)) then

				next_flag_Z := (unsigned(next_A) = 0);
			end if;
						
			-- Assert the new state, program counter, and register values for the next clock 
			-- cycle.
			prog_cntr <= next_pc; 
			state <= next_state;					
			reg_A <= next_A;
			reg_SP <= next_SP;
			flag_C <= next_flag_C;
			flag_Z <= next_flag_Z;
			flag_I <= next_flag_I;
			RISRV <= next_RISRV;
		end if;
	end process;

	-- Here we have combinatorial logic that produce various MCU signals.
	MCU_MUX : process (all) is
	begin		
		alu_cin <= false;
		alu_ctrl <= alu_cmd_add;
		mcu_addr <= operand;
		mcu_data_out <= reg_A;
		WDS <= false;
		
		if IRQ and (not flag_I) and (state = read_opcode) then
			mcu_addr <= reg_SP;
			mcu_data_out <= (others => '0');
			mcu_data_out(addr_len-1 downto 0) <= prog_cntr;
			WDS <= true;
		else
			case opcode is
			
			when ld_m_A =>
				WDS <= true;
			
			when add_A_m =>
				alu_cin <= false;
				alu_ctrl <= alu_cmd_add;
					
			when adc_A_m =>
				alu_cin <= flag_C;
				alu_ctrl <= alu_cmd_add;
					
			when sub_A_m =>
				alu_cin <= false;
				alu_ctrl <= alu_cmd_sub;
					
			when sbc_A_m =>
				alu_cin <= flag_C;
				alu_ctrl <= alu_cmd_sub;

			when and_A_m =>
				alu_ctrl <= alu_cmd_and;
			
			when or_A_m =>
				alu_ctrl <= alu_cmd_or;
							
			when xor_A_m =>
				alu_ctrl <= alu_cmd_xor;
				
			when rr_A =>
				alu_ctrl <= alu_cmd_rr;
				
			when rl_A =>
				alu_ctrl <= alu_cmd_rl;
			
			when push_A =>
				mcu_addr <= reg_SP;
				mcu_data_out <= reg_A;
				WDS <= true;

			when push_F =>
				mcu_addr <= reg_SP;
				mcu_data_out <= (others => '0');
				mcu_data_out(7) <= to_std_logic(flag_I);
				mcu_data_out(2) <= to_std_logic(flag_C);
				mcu_data_out(0) <= to_std_logic(flag_Z);
				WDS <= true;
				
			when call_n =>
				mcu_addr <= reg_SP;
				mcu_data_out <= (others => '0');
				mcu_data_out(addr_len-1 downto 0) <= prog_cntr;
				if (state = read_opcode) then
					WDS <= true;
				end if;

			when pop_A | pop_F | ret_cll | ret_int =>
				mcu_addr <= reg_SP;
				
			when others =>
				null;
			
			end case;
		end if;
	end process;
	
	-- The ISRV Delay takes RISRV generated on the rising edge of CK
	-- and delays until the falling edge, so as to match signals written
	-- to registers by the MCU. The resulting ISRV signal can be used
	-- to boost the MCU clock in the same way that a write to a BOOST
	-- and ENFCK register can do the same.
	ISRV_Delay : process (RESET, CK) is
	begin
		if (RESET = '1') then
			ISRV <= false;
		elsif falling_edge(CK) then
			ISRV <= RISRV;
		end if;
	end process;
end behavior;

