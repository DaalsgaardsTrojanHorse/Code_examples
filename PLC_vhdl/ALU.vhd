library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.std_logic_unsigned.all;
use IEEE.NUMERIC_STD.ALL;

entity ALU is
    Port( 
    A, B: in std_logic_vector(15 downto 0);
    ALU_control: in std_logic_vector(7 downto 0); -- 1 input for 6 control inputs
    ALU_out: out std_logic_vector(15 downto 0);
    c_out: out std_logic;
    clk: in std_logic
    );
end ALU;

architecture Behavioral of ALU is
    signal F_i: std_logic_vector(16 downto 0);
    signal ALU_select: std_logic_vector(7 downto 0);
begin
ALU_out <= F_i(15 downto 0);
c_out <= F_i(16);
ALU_select <= ALU_control;

-- Process for ALU decoder
p1: process(clk)
begin
    if (rising_edge(clk)) then
        case (ALU_select) is -- case of the useful combinations for ALU
            when x"00" => F_i <= std_logic_vector(unsigned(('0' & A)) + unsigned(('0' & B)));
            when x"01" => if (A >= B) then
                          F_i <= std_logic_vector(unsigned(('0' & A)) - unsigned(('0' & B)));
                          else
                          F_i <= (others => '0');
                          end if;
            when x"02" => F_i <= std_logic_vector(unsigned(('0' & A)) - 1);
            when x"03" => F_i <= std_logic_vector(unsigned(('0' & A)) + 1);
            when x"04" => F_i <= '0' & (A and B);
            when x"05" => F_i <= '0' & (A or B);
            when x"06" => F_i <= '0' & (not A);
            when x"07" => F_i <= '0' & (A xor B);
            when others => NULL;                
        end case;
    end if;
end process p1;
    
end Behavioral;