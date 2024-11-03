library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;


entity IO_PORTS is
    Port (  clk : in STD_LOGIC;
            DDR : in std_logic_vector(7 downto 0);
            PortTx : in std_logic_vector(7 downto 0);
            PortRx : out std_logic_vector(7 downto 0);
            IO : inout std_logic_vector(7 downto 0)
            );
end IO_PORTS;

architecture Behavioral of IO_PORTS is



begin

Transmit:  
for i in 0 to 7 generate
    IO(i) <= PortTx(i) when DDR(i) = '1' else 'Z';
end generate;
            


inputPort: process(clk)
    begin
        if (rising_edge(clk)) then
            for i in 0 to 7 loop
            PortRx(i) <= IO(i);
            end loop;
        end if;
end process inputPort;
end Behavioral;
