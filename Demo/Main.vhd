-- RISC Microcontroller, Five-Bit Opcodes (RMC5) Demonstration Program

-- V1.0 [28-OCT-26] The microcontroller data bus and control signals are connected to 
-- input-output pins, LCMXO2-256ZE in TQFP-100.


library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	port (
		prog_cntr : out std_logic_vector(6 downto 0);
		prog_data : in std_logic_vector(11 downto 0);
		mcu_data : inout std_logic_vector(7 downto 0);
		mcu_addr : out std_logic_vector(6 downto 0);
		MCUWR, MCUDS, IRSV : out std_logic;
		CK, IRQ, RESET : in std_logic
	);
end;

architecture behavior of main is

-- Microcontroller data in and out.
	signal mcu_data_in, mcu_data_out : std_logic_vector(7 downto 0);
	signal wr_boolean, ds_boolean, irsv_boolean : boolean;

-- Attributes that request signals to be retained instead of distributed.
	attribute syn_keep : boolean;
	attribute nomerge : string;
	
-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;

begin

-- The Bus Controller decides when to drive the data lines.
	mcu_data <= mcu_data_out when MCUWR else (others => 'Z');
	mcu_data_in <= mcu_data;
	MCUWR <= to_std_logic(wr_boolean);
	MCUDS <= to_std_logic(ds_boolean);
	IRSV <= to_std_logic(irsv_boolean);	

-- The RMC5, configured for this application by its generic map.
	MCU : entity RMC5_MCU
		port map (
			prog_cntr => prog_cntr,
			prog_data => prog_data,
			mcu_data_out => mcu_data_out,
			mcu_data_in => mcu_data_in,
			mcu_addr => mcu_addr,
			WR => wr_boolean,
			DS => ds_boolean,
			IRQ => (IRQ = '1'),
			ISRV => irsv_boolean,
			RESET => RESET,
			CK => CK
		);
end behavior;