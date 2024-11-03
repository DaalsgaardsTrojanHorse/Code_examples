library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;


entity top is

GENERIC (
    data_width_instruction: INTEGER := 40; --bit width for instruction
    data_width_mem: INTEGER := 16; --bit width for memory
    data_length_mem: INTEGER := 16; --size of memory
    max_stack_size: INTEGER := 6 -- maximum size of stack before it overides GPR and other predefined registers(set to 1 higher than registers)
);

    PORT ( 
    clk: in STD_LOGIC;
    reset: in STD_LOGIC;   
    IO: inout STD_LOGIC_VECTOR(7 DOWNTO 0); 
    OUT_DEBUG: OUT std_logic_vector(15 downto 0);
    Sel: in std_logic_vector(7 downto 0);
    SDA : inout std_logic;
    SCL : in std_logic;
    I2C_EN : in std_logic
    );
end top;


architecture Behavioral of top is

COMPONENT ALU -- PORT map til ALU
Port( 
    A, B: in std_logic_vector(15 downto 0);
    ALU_control: in std_logic_vector(7 downto 0); -- 1 input for 6 control inputs
    ALU_out: out std_logic_vector(15 downto 0);
    c_out: out std_logic;
    clk: in std_logic
    );
end component;

--ALU internal signals
signal regAIN, regBIN : std_logic_vector (15 downto 0);
signal ALUselect : std_logic_vector (7 downto 0);
--Flags
signal c_flag, z_flag: std_logic;
signal Flags: std_logic_vector(15 downto 0);

component IO_PORTS -- port map til IO_PORTS
Port (  clk : in STD_LOGIC;
        DDR : in std_logic_vector(7 downto 0);
        PortTx : in std_logic_vector(7 downto 0);
        PortRx : out std_logic_vector(7 downto 0);
        IO : inout std_logic_vector(7 downto 0)
        );
end component;
signal DDR, PORTTX, PORTRX: std_logic_vector (7 downto 0) := (others => '0'); 

COMPONENT I2C 
PORT (
      SCL : in STD_LOGIC;
      SDA : inout STD_LOGIC;
      I2C_EN : in STD_LOGIC;
      BYTE_OUT : out std_logic_vector(7 downto 0);
      DATA_RECEIVED : out UNSIGNED(7 downto 0);
      Clk : in STD_LOGIC);
END COMPONENT;

--Declar state for loading I2C into program mem
TYPE STATE_LOAD_I2C is (S_IDLE, S_START, S_CHECK, S_STOP, L_OPCODE, L_ADDR1_1, L_ADDR1_2, L_ADDR2_1, L_ADDR2_2);
SIGNAL STATE_I2C, NEXT_S : STATE_LOAD_I2C; 
SIGNAL WAIT_DATA         : std_logic            := '0';
SIGNAL CNT_INS           : unsigned(7 downto 0) := (others => '0');
SIGNAL INS_NR            : unsigned(7 downto 0) := (others => '0');
SIGNAL DATA_READY        : std_logic;
SIGNAL D_CNT             : unsigned(7 downto 0);
signal BYTE_RECEIVE      : std_logic_vector(7 downto 0) := (others => '0');





--Memory
type prog_mem_type is array(natural range <>) of std_logic_vector(data_width_instruction - 1 downto 0);
type ram_mem_type is array(natural range <>) of std_logic_vector(data_width_mem - 1 downto 0);
signal ram_mem: ram_mem_type (0 to data_length_mem - 1);
signal prog_mem: prog_mem_type (0 to data_length_mem - 1) := 
--(x"1400000003", x"1400010005", x"270000000B", x"0000000001", x"1000000004", x"0500020003", x"1000000004", x"0400020003", 
--x"1100000000", x"1100000001", x"2000000000", x"120000000F", x"12000000F0", x"1100000002", x"1100000003", x"2800000000"); --Case 2 test af push
(x"120000AAAA", x"FE00000003", x"120000BBBB", x"FE00000001" , x"120000CCCC", x"14000600F0", x"FE00000001", x"140006000F", 
x"FE00000001", x"0300080000", x"1300080004", x"2100080000", x"1400060055", x"FE00000001", x"2000000002", x"FF00000000"); --Case 1 test af Mov, loop

signal addr1, addr2: unsigned (15 downto 0);
signal opcode: std_logic_vector(7 downto 0);
signal RE, WE: std_logic;

subtype ram_mstype is ram_mem_type(0 to 7+data_length_mem); 

signal RAM_mirror : ram_mstype; 



--Registre
signal ACC_ALU : std_logic_vector(data_width_mem - 1 downto 0); --output ALU
signal status_reg: std_logic_vector(data_width_mem - 1 downto 0);
signal IR: std_logic_vector (data_width_instruction - 1 downto 0);

--Pointers
signal PC: integer range 0 to data_length_mem - 1 := 0;
signal SP: integer range 0 to data_length_mem - 1 := (data_length_mem - 1);


--i_state to control state
type i_state_type is (ist1, ist2, ist3, ist4, ist5, ist6);
signal i_state : i_state_type := ist1;


--Program variabler
signal startup : std_logic := '0';
signal klk: std_logic := '0';
signal klk_reg: unsigned(23 downto 0) := (others => '0'); 
signal sleep_counter: unsigned(31 downto 0):= (others => '0');
----------------------------SETUP for program------------------------------------
begin
Flags(0) <= c_flag; 

Flags(1) <= z_flag; 

RAM_mirror(0) <= std_logic_vector(to_unsigned(integer(PC),16)); 

RAM_mirror(1) <= std_logic_vector(opcode) & x"00"; 

RAM_mirror(2) <= std_logic_vector(to_unsigned(TO_INTEGER(addr1),16)); 

RAM_mirror(3) <= std_logic_vector(to_unsigned(TO_INTEGER(addr2),16)); 

RAM_mirror(4) <= std_logic_vector(to_unsigned(integer(SP),16)); 

RAM_mirror(5) <= Flags;  

RAM_mirror(6+0 TO 6+data_length_mem - 1) <= ram_mem(0 TO data_length_mem - 1); 

 

Switch_case_debug : process(clk,Sel, RAM_mirror) is 

begin 
if(rising_edge(clk)) then
    case Sel is 
        when "00000000" => 

           OUT_DEBUG <= RAM_mirror(0);  

        when "00000001" => 

            OUT_DEBUG <= RAM_mirror(1);   

        when "00000010" => 

            OUT_DEBUG <= RAM_mirror(2);   

        when "00000011" => 

            OUT_DEBUG <= RAM_mirror(3);   

        when "00000100" => 

            OUT_DEBUG <= RAM_mirror(4);   

        when "00000101" => 

            OUT_DEBUG <= RAM_mirror(5);   

        when "00000110" => 

            OUT_DEBUG <= RAM_mirror(6);   

        when "00000111" => 

            OUT_DEBUG <= RAM_mirror(7);   

        when "00001000" => 

            OUT_DEBUG <= RAM_mirror(8);   

        when "00001001" => 

            OUT_DEBUG <= RAM_mirror(9);   

        when "00001010" => 

            OUT_DEBUG <= RAM_mirror(10);   

        when "00001011" => 

            OUT_DEBUG <= RAM_mirror(11);   

        when "00001100" => 

            OUT_DEBUG <= RAM_mirror(12);   

        when "00001101" => 

            OUT_DEBUG <= RAM_mirror(13);   

        when "00001110" => 

            OUT_DEBUG <= RAM_mirror(14);   

        when "00001111" => 

            OUT_DEBUG <= RAM_mirror(15);   

        when "00010000" => 

            OUT_DEBUG <= RAM_mirror(16);   

        when "00010001" => 

            OUT_DEBUG <= RAM_mirror(17);   

        when "00010010" => 

            OUT_DEBUG <= RAM_mirror(18);   

        when "00010011" => 

            OUT_DEBUG <= RAM_mirror(19);   

        when "00010100" => 

            OUT_DEBUG <= RAM_mirror(20);   

        when "00010101" => 

            OUT_DEBUG <= RAM_mirror(21);   

        when "00010110" => 

            OUT_DEBUG <= RAM_mirror(22);   

        when others => 

            OUT_DEBUG <= (others => '0');  

    end case;     
end if;
end process;

--for ALU portmap
ALUi : ALU
    Port map(
        A => regAIN,
        B => regBIN,
        ALU_control => ALUselect,
        ALU_out => ACC_ALU, --ACCUMULATER
        c_out => c_flag,
        clk => clk
    );
    
IOi : IO_PORTS
    PORT map(
        clk => clk,
        IO => IO,
        PortTx => PORTTX,
        PortRx => PORTRX,
        DDR => DDR
    );

I2C_SLAVE : I2C
PORT MAP(SCL => SCL,
         SDA => SDA,
         Clk => clk,
         I2C_EN => I2C_EN,
         DATA_RECEIVED => D_CNT,
         BYTE_OUT => BYTE_RECEIVE
);

    
   
--registers names to registers
register_map: process (clk)
begin
    if (falling_edge(clk)) then
        --ram_mem(4) <= ACC_ALU; just a note
        DDR <= ram_mem(5)(7 downto 0);  -- set whether its input or output (direct data register)
        PORTTX <= ram_mem(6)(7 downto 0); --transmit data out on port   
        --ram_mem(7)(7 downto 0) <= PORTRX; --recaicve data in on port
    end if;
end process register_map;




--clk laver om til klk for at nedskalere programklok
clock_divider: process (clk)
begin
    if (clk'event and clk = '1') then 
        if (klk_reg < 12000000) then 
            klk_reg <= klk_reg + 1;
        else 
            klk_reg <= (others => '0'); 
            klk <= NOT klk;
        end if;
    end if;
end process clock_divider;


--opdatere addr1 og addr2
addr_update: process (clk)
    begin
        if (rising_edge(clk)) then
            addr1 <= unsigned(IR(31 downto 16));
            addr2 <= unsigned(IR(15 downto 0));
            opcode <= IR(39 downto 32);
        end if;
end process addr_update;
    


--TEST stack til waveform
--process (clk)
--    begin
--        if (rising_edge(clk)) then
--            stack1 <= ram_mem(data_length_mem - 1);
--            stack2 <= ram_mem(data_length_mem - 2);
--            stack3 <= ram_mem(data_length_mem - 3);
--            stack4 <= ram_mem(data_length_mem - 4);
--        end if;
--end process;
    

----------------------------Begin of program------------------------------------
cpu: process (klk)
--Procedure for reset af program
procedure nulstil is 
    begin
        IR <= prog_mem(0);
        PC <= 1;
        SP <= data_length_mem - 1; 
        i_state <= ist1;
end procedure nulstil;

--Procedure for standard instructioner
procedure std_end_instruction is
    begin 
        IR <= prog_mem(PC);
        PC <= PC + 1;
        i_state <= ist1;
end procedure std_end_instruction;


--------------------------CASE FOR INSTRUKTIONER-------------------------------
BEGIN
    if (rising_edge(klk)) then
        if (startup = '0') then
            IR <= prog_mem(0);
            PC <= 1;
            SP <= data_length_mem - 1;       
            startup <= '1';   
            
        elsif (reset = '1') then 
            nulstil;
            
        elsif (PC = data_length_mem) then
            PC <= 0;        
            
        else

            
            case (opcode) is

  
   ----------------------------ALU intruktioner (x"0...")--------------------------------------------------  
   
   
             
                --ALU opcodes--
                when x"00"|x"01"|x"02"|x"03"|x"04"|x"05"|x"06"|x"07" =>
                    ALUselect <= IR(39 downto 32);
                    case (i_state) is
                        when ist1 =>
                            regAIN <= ram_mem(to_integer(addr1));
                            regBIN <= ram_mem(to_integer(addr2));
                            i_state <= ist2;
                        when ist2 =>
                            ram_mem(4) <= ACC_ALU;
                            std_end_instruction;
                        when others =>   
                    end case;
                    
                    
  ----------------------------Data intruktioner (x"1...")--------------------------------------------------      
  
             
                --PUSH
                --Push a variable from register to stack
                when x"10" =>
                    case (i_state) is 
                        when ist1 =>
                            if (SP = max_stack_size) then
                                std_end_instruction;
                            else
                                ram_mem(SP) <= ram_mem(to_integer(addr2));
                                SP <= SP - 1; 
                                i_state <= ist2;
                            end if;
                        when ist2 =>
                            std_end_instruction;    
                        when others =>  
                    end case;
                    
                --POP
                --Pop a varible from stack and store in register
                when x"11" =>
                    case (i_state) is 
                        when ist1 =>
                            if (SP = data_length_mem - 1)    then 
                                ram_mem(to_integer(addr2)) <= ram_mem(SP);                                
                                ram_mem(SP) <= x"0000";
                                i_state <= ist2;
                            else
                                ram_mem(to_integer(addr2)) <= ram_mem(SP + 1);
                                ram_mem(SP + 1) <= x"0000";
                                SP <= SP + 1;
                                i_state <= ist2;
                            end if;    
                        when ist2 =>
                            std_end_instruction;
                        when others =>   
                    end case;
                
                
                --intPUSH
                -- Push an integer ontop of stack
                when x"12" =>          
                    case (i_state) is 
                        when ist1 =>
                            if (SP = max_stack_size) then
                                std_end_instruction;
                            else
                                ram_mem(SP) <= std_logic_vector(addr2);
                                SP <= SP - 1; 
                                i_state <= ist2;
                            end if;
                        when ist2 =>
                            std_end_instruction;
                        when others =>  
                    end case;
                    
                    
                    
                --MOVreg
                -- Move register to register
                when x"13" =>
                    case (i_state) is
                        when ist1 =>
                            ram_mem(to_integer(addr1)) <= ram_mem(to_integer(addr2));
                            i_state <= ist2;
                        when ist2 =>
                            std_end_instruction;
                        when others =>   
                    end case;
                    
                    
                    
                --MOVc
                -- move a constant to register
                when x"14" =>
                    case (i_state) is
                        when ist1 =>
                            ram_mem(to_integer(addr1)) <= std_logic_vector(addr2);
                            i_state <= ist2;
                        when ist2 =>
                            std_end_instruction;
                        when others =>   
                    end case;
                
               
 ----------------------------Branch intruktioner (x"2...")--------------------------------------------------          
               
               --JMP
               --jump to a specific address in prog_mem
               when x"20" =>
                    case (i_state) is
                        when ist1 =>
                            IR <= prog_mem(to_integer(addr2)); -- IR is address of the subroutine
                            PC <= (to_integer(addr2) + 1);   
                        when others =>
                    end case;
               
               --JR
               --Jump to an address relative to where IR is
               when x"21" =>
                    case (i_state) is
                        when ist1 =>
                            if (addr1 > 0) then
                                IR <= prog_mem(PC - to_integer(addr1) - 1); -- IR is address of the subroutine
                                PC <= (PC - to_integer(addr1)); 
                            elsif (addr2 > 0) then
                                IR <= prog_mem(PC + to_integer(addr2) - 1); -- IR is address of the subroutine
                                PC <= (PC + to_integer(addr2));
                            else
                                std_end_instruction;
                            end if;   
                        when others =>
                    end case;
               
               --CMP
               --compare two registers and set zero flag high if they are 0
               when x"22" =>
                    case (i_state) is
                        when ist1 =>
                            if (unsigned((ram_mem(to_integer(addr1)))) - unsigned((ram_mem(to_integer(addr2)))) = 0) then 
                                z_flag <= '1';
                            else
                                z_flag <= '0';
                            end if;
                            i_state <= ist2;
                        when ist2 =>
                            std_end_instruction;
                        when others =>
                    end case;
               
               
               --JZ
               --jump if zero flag is set to 1   
               when x"23" =>
                   case (i_state) is
                        when ist1 =>
                            if (z_flag = '1') then
                                IR <= prog_mem(to_integer(addr2)); 
                                PC <= (to_integer(addr2) + 1); 
                            else
                                std_end_instruction; 
                            end if; 
                        when others =>
                    end case;
               
               --JNZ
               --jump if zero flag is set to 0  
               when x"24" =>
                    case (i_state) is
                        when ist1 =>
                            if (z_flag = '0') then
                                IR <= prog_mem(to_integer(addr2)); 
                                PC <= (to_integer(addr2) + 1); 
                            else
                                std_end_instruction; 
                            end if;  
                        when others =>
                    end case;     
                    
                    
               --JC
               --jump if Carry flag is set to 1   
               when x"25" =>
                    case (i_state) is
                        when ist1 =>
                            if (c_flag = '1') then
                                IR <= prog_mem(to_integer(addr2)); -- IR is address of the subroutine
                                PC <= (to_integer(addr2) + 1); 
                            else
                                std_end_instruction; 
                            end if;  
                        when others =>
                    end case;
               
               --JNC
               --jump if carry flag is set to 0  
               when x"26" =>
                    case (i_state) is
                        when ist1 =>
                            if (c_flag = '0') then
                                IR <= prog_mem(to_integer(addr2)); -- IR is address of the subroutine
                                PC <= (to_integer(addr2) + 1); 
                            else
                                std_end_instruction; 
                            end if;  
                        when others =>
                    end case;
              
               
               --CALL
               --Jumps to a subroutine
               when x"27" =>
                    case (i_state) is
                        when ist1 =>
                            ram_mem(SP) <= std_logic_vector(to_unsigned(PC, data_length_mem)); --IR
                            ram_mem(SP - 1) <= std_logic_vector(to_unsigned(PC + 1, data_length_mem)); --PC
                            SP <= SP - 2;
                            i_state <= ist2;
                        when ist2 =>
                            IR <= prog_mem(to_integer(addr2));
                            PC <= (to_integer(addr2) + 1);
                            i_state <= ist1;
                        when others =>
                    end case;
               
               
               --RET
               --Return to the next instruction from CALL
               when x"28" =>
                    case (i_state) is
                        when ist1 =>
                            IR <= prog_mem(to_integer(unsigned((ram_mem(SP + 2)))));
                            PC <= to_integer(unsigned(ram_mem(SP + 1)));
                            ram_mem(SP + 1) <= x"0000";
                            ram_mem(SP + 2) <= x"0000";
                            SP <= SP + 2;               
                        when others =>
                    end case;            
               
 ----------------------------MCU control intruktioner (x"F...")--------------------------------------------------               
               
               --SLEEP
               --stops the program for x clock cycles
               when x"FE" =>
                    case (i_state) is 
                        when ist1 =>
                            if (sleep_counter < unsigned(addr1 & addr2)) then
                                sleep_counter <= sleep_counter + 1;
                            elsif (sleep_counter >= unsigned(addr1 & addr2)) then
                                sleep_counter <= (others => '0');
                                std_end_instruction;
                            end if;  
                        when others =>
                    end case;  
               
               
               --NOP
               -- NO Operation
               when x"FF" =>     
                    case (i_state) is
                        when ist1 =>
                            std_end_instruction;
                        when others =>   
                    end case;           
                when others => 
            end case;
        end if;
    end if;
end process cpu;


---LOAD DATA INTO PROGRAM MEM DO NOT WORK AT THE GIVEN TIME----


I2C_READY : process(Clk)
begin
    if(rising_edge(Clk)) then
        if(D_CNT = 8) then
            DATA_READY <= '1';
        elsif(D_CNT = 4) then 
            DATA_READY <= '0';
        end if;
    end if;
end process;


UPDATE_STATE : process(Clk, I2C_EN) 
begin
    if(I2C_EN ='0') then
        STATE_I2C <= S_IDLE;
    elsif(rising_edge(Clk)) then
        STATE_I2C <= NEXT_S;
    end if;
end process;

Load_I2C_2_pMEM : process(Clk, I2C_EN)
begin    
if(rising_edge(Clk)) then   
    CASE STATE_I2C is
    
    when S_IDLE  =>

    if(CNT_INS = INS_NR) then        
        WAIT_DATA         <= '0';
        CNT_INS           <= (others => '0');
        INS_NR            <= (others => '0');
    
        if(I2C_EN = '0') then
            NEXT_S        <= S_IDLE;
        else    
            NEXT_S        <= S_START;
        end if;
    else 
        PROG_MEM          <= (others => (others => '1'));
        CNT_INS           <= (others => '0');
        INS_NR            <= (others => '0');
    end if;
    
    when S_START =>
    if(DATA_READY = '0') then
        NEXT_S        <= S_START;
    elsif(DATA_READY = '1' and WAIT_DATA = '0') then
        INS_NR        <= "00000001";--UNSIGNED(BYTE_RECEIVE);
        NEXT_S        <= L_OPCODE;
        WAIT_DATA     <= '1';
    else 
        NEXT_S        <= S_START;
    end if;
---------Loading state------------------           
        when L_OPCODE  =>
        if(DATA_READY = '1' and WAIT_DATA = '0') then
            PROG_MEM(TO_INTEGER(CNT_INS))(39 downto 32) <= BYTE_RECEIVE;
            NEXT_S                                      <= L_ADDR1_1;
            WAIT_DATA                                   <= '1';
        elsif(DATA_READY = '0' and WAIT_DATA = '1') then
            NEXT_S                                      <= L_OPCODE; 
            WAIT_DATA                                   <= '0';
        else 
            NEXT_S                                      <= L_OPCODE;
        end if;
        
        when L_ADDR1_1 =>
        if(DATA_READY = '1' and WAIT_DATA = '0') then
            PROG_MEM(TO_INTEGER(CNT_INS))(31 downto 24) <= BYTE_RECEIVE;
            NEXT_S                                      <= L_ADDR1_2;
            WAIT_DATA                                   <= '1';
        elsif(DATA_READY = '0' and WAIT_DATA = '1') then
            NEXT_S                                      <= L_ADDR1_1;
            WAIT_DATA                                   <= '0';
        else
            NEXT_S                                      <= L_ADDR1_1;
        end if;            
        
        when L_ADDR1_2 =>

        if(DATA_READY = '1' and WAIT_DATA = '0') then
            PROG_MEM(TO_INTEGER(CNT_INS))(23 downto 16) <= BYTE_RECEIVE;
            NEXT_S                                      <= L_ADDR2_1;
            WAIT_DATA                                   <= '1';
        elsif(DATA_READY = '0' and WAIT_DATA = '1') then
            NEXT_S                                      <= L_ADDR1_2;
            WAIT_DATA                                   <= '0';
        else
            NEXT_S                                      <= L_ADDR1_2;
        end if;
                    
        when L_ADDR2_1 =>

        if(DATA_READY = '1' and WAIT_DATA = '0') then
            PROG_MEM(TO_INTEGER(CNT_INS))(15 downto 8) <= BYTE_RECEIVE;
            NEXT_S                                     <= L_ADDR2_2;
            WAIT_DATA                                  <= '1';
        elsif(DATA_READY = '0' and WAIT_DATA = '1') then 
            NEXT_S                                     <= L_ADDR2_1;
            WAIT_DATA                                  <= '0';
        else
            NEXT_S                                     <= L_ADDR2_1;
        end if;
            
        when L_ADDR2_2 =>

        if(DATA_READY = '1' and WAIT_DATA = '0') then
            PROG_MEM(TO_INTEGER(CNT_INS))(7 downto 0) <= BYTE_RECEIVE;
            NEXT_S                                    <= S_CHECK;
            WAIT_DATA                                 <= '1';
        elsif(DATA_READY = '0' and WAIT_DATA = '1') then
            NEXT_S                                    <= L_ADDR2_2;
            WAIT_DATA                                 <= '0';
        else
            NEXT_S                                    <= L_ADDR2_2;
        end if; 
--Check if its done loading data into the instructions or it should keep going           
    when S_CHECK =>
    --I2C_Transfer_Done   <= '1';
        if(CNT_INS < INS_NR ) then
            CNT_INS       <= CNT_INS + 1;
            NEXT_S      <= L_OPCODE;
        elsif(CNT_INS = INS_NR) then
            NEXT_S      <= S_STOP;
        end if;
--STATE_STOP CAN FIRST STOP WHEN THE PIN HAS BEEN PULLED LOW!        
    when S_STOP  =>
    if(I2C_EN = '0') then
        NEXT_S          <= S_STOP;
    else
        NEXT_S          <= S_IDLE;
    end if;
    
    END CASE;
end if;
end process;

end Behavioral;
