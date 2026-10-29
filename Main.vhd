-- A3054L Long-Life IPT Firmware, Top-Level Unit

-- V2.0 [28-OCT-26] Based upon P3054 V1.9. Creating long-life version.


library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	port (
		cpu_data : inout std_logic_vector(7 downto 0);
		cpu_addr : out std_logic_vector(6 downto 0);
		CPUWR, CPUDS, IRSV : out std_logic;
		CK, IRQ, RESET : in std_logic
	);
end;

architecture behavior of main is

-- CPU data in and out.
	signal cpu_data_in, cpu_data_out : std_logic_vector(7 downto 0);
	signal wr_boolean, ds_boolean, irsv_boolean : boolean;

-- Attributes that request signals to be retained instead of distributed.
	attribute syn_keep : boolean;
	attribute nomerge : string;
	
-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;

begin

-- The Bus Controller decides when to drive the data lines.
	cpu_data <= cpu_data_out when CPUWR else (others => 'Z');
	cpu_data_in <= cpu_data;
	CPUWR <= to_std_logic(wr_boolean);
	CPUDS <= to_std_logic(ds_boolean);
	IRSV <= to_std_logic(irsv_boolean);	

-- The RMC5, configured for this application by its generic map.
	MCU : entity RMC5_CPU
		port map (
			cpu_data_out => cpu_data_out,
			cpu_data_in => cpu_data_in,
			cpu_addr => cpu_addr,
			WR => wr_boolean,
			DS => ds_boolean,
			IRQ => (IRQ = '1'),
			ISRV => irsv_boolean,
			RESET => RESET,
			CK => CK
		);
end behavior;