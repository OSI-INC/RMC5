-- RISC Microcontroller, Five-Bit Opcodes (RMC5) Demonstration Program

-- V1.0 [28-OCT-26] The microcontroller data bus and control signals are connected to 
-- input-output pins, LCMXO2-256ZE in TQFP-100.


library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	generic (
		addr_len : positive := 9; -- Can be five to sixteen
		data_len : positive := 8; -- Must be eight
		instr_len : positive := addr_len + 5
	);
	port (
		mcu_data : inout std_logic_vector(data_len-1 downto 0);
		mcu_addr : out std_logic_vector(addr_len-1 downto 0);
		WDS : inout std_logic;
		IRSV : out std_logic;
		CK, IRQ, RESET : in std_logic
	);
end;

architecture behavior of main is

-- Microcontroller data in and out.
	signal mcu_data_in, mcu_data_out : std_logic_vector(data_len-1 downto 0);
	signal wds_boolean, irsv_boolean : boolean;
	signal prog_cntr : std_logic_vector(addr_len-1 downto 0);
	signal prog_data : std_logic_vector(instr_len-1 downto 0);

-- Attributes that request signals to be retained instead of distributed.
	attribute syn_keep : boolean;
	attribute nomerge : string;
	
-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;

begin

-- The Bus Controller decides when to drive the data lines.
	mcu_data <= mcu_data_out when WDS else (others => 'Z');
	mcu_data_in <= mcu_data;
	WDS <= to_std_logic(wds_boolean);
	IRSV <= to_std_logic(irsv_boolean);	

-- The instruction memory
	Instruction_Memory : entity ROM
		port map (
			Address => prog_cntr,
			OutClock => not CK,
			OutClockEn => '1',
			Reset => '0',
			Q => prog_data
		);
		
-- The RMC5, configured for this application by its generic map.
	Microcontroller : entity RMC5_MCU
		generic map (
			addr_len => addr_len
		)
		port map (
			prog_cntr => prog_cntr,
			prog_data => prog_data,
			mcu_data_out => mcu_data_out,
			mcu_data_in => mcu_data_in,
			mcu_addr => mcu_addr,
			WDS => wds_boolean,
			IRQ => (IRQ = '1'),
			ISRV => irsv_boolean,
			RESET => RESET,
			CK => CK
		);
end behavior;