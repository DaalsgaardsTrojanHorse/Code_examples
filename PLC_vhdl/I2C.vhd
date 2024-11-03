library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

entity I2C is
    Port ( 
           SCL : in STD_LOGIC;
           SDA : inout STD_LOGIC;
           I2C_EN : in STD_LOGIC;
           BYTE_OUT : out std_logic_vector(7 downto 0);
           DATA_RECEIVED : out unsigned(7 downto 0);
           Clk : in STD_LOGIC
           );
end I2C;

architecture Behavioral of I2C is

type STATE_I2C is (STATE_IDLE, STATE_ADDRESS, STATE_ACK_ADDRESS, STATE_READ_BYTE, STATE_ACK_BYTE, STATE_STOP);
signal STATE, NEXT_STATE: STATE_I2C;

--SYNC SIGNALS TO SYNC SDA AND SCL WITH THE CLOCK
signal SDA_ASYNC, SDA_BUS, SCL_ASYNC,SCL_BUS, SDA_PREV, SCL_PREV, SCL_DELAY, SDA_DELAY  : std_logic;
signal SDA_RISING, SCL_RISING, SDA_FALLING, SCL_FALLING : boolean;


signal ADDRESS              : std_logic_vector(7 downto 0);
signal DATA_IN              : std_logic_vector(7 downto 0);
signal COUNTER              : integer range 0 to 8         := 0;
signal COUNTER_BYTE         : integer range 0 to 8         := 0;
signal RECIEVED_DATA        : std_logic                    := '1';
signal SDA_VALUE, SCL_VALUE : std_logic                    := '1';
signal SLAVE_ADDRESS        : std_logic_vector(7 downto 0) := "01110000";
signal ACK_SIG_SEND         : std_logic                    := '0';
signal ACK_SEND             : std_logic                    := '0';
signal I2C_OUT              : std_logic                    := '0';
--__________________________________________________________--


begin

SDA <= ACK_SEND when I2C_OUT = '1' else 'Z';

BYTE_OUT <= DATA_IN; 


State_update : process(Clk, I2C_EN)
begin
    if(I2C_EN = '0') then
        STATE <= STATE_IDLE;
    elsif(rising_edge(Clk)) then
        STATE <= NEXT_STATE;
    end if;
end process;

Sync_SCL_SDA: process(Clk, I2C_EN, SCL, SDA) 
begin
    if(I2C_EN = '0') then
        SCL_ASYNC   <= '1';
        SCL_BUS     <= '1';
        SCL_PREV    <= '1';
           
        SDA_ASYNC   <= '1';
        SDA_BUS     <= '1';
        SDA_PREV    <= '1';
    elsif (rising_edge(Clk)) then
        SCL_ASYNC   <= SCL;
        SCL_BUS     <= SCL_ASYNC;
        SCL_DELAY   <= SCL_BUS;
        SCL_PREV    <= SCL_DELAY;
            
        SCL_RISING  <= SCL_BUS = '1' and SCL_PREV = '0';
        SCL_FALLING <= SCL_BUS = '0' and SCL_PREV = '1';
            
        SDA_ASYNC   <= SDA;            
        SDA_BUS     <= SDA_ASYNC;
        SDA_PREV    <= SDA_BUS;
            
        SDA_RISING  <= SDA_BUS = '1' and SDA_PREV = '0';
        SDA_FALLING <= SDA_BUS = '0' and SDA_PREV = '1';          
    end if;
end process Sync_SCL_SDA;

SCL_STATE : process(CLK, I2C_EN, SCL_VALUE)
begin
    if(rising_edge(Clk)) then
        if(I2C_EN = '0') then
            SCL_VALUE <= '1';
        elsif(SCL_RISING = TRUE) then
            SCL_VALUE <= '1';
        elsif(SCL_FALLING = TRUE) then
            SCL_VALUE <= '0';
        else
            SCL_VALUE <= SCL_VALUE;
        end if;
    end if;
end process;



SDA_STATE : process(CLK, I2C_EN, SDA_VALUE)
begin
    if(rising_edge(Clk)) then
        if(I2C_EN = '0') then
            SDA_VALUE <= '1';
        elsif(SDA_RISING = TRUE) then
            SDA_VALUE <= '1';
        elsif(SDA_FALLING = TRUE) then
            SDA_VALUE <= '0';
        else
            SDA_VALUE <= SDA_VALUE;
        end if;
    end if;
end process;

-----------------------------------------------------
-- FSM  
-----------------------------------------------------

I2C_STATE_MACHINE : process(Clk, STATE, SDA_VALUE, SDA_FALLING,SCL_FALLING, SCL_VALUE, ADDRESS, COUNTER, RECIEVED_DATA, ACK_SIG_SEND, I2C_OUT, ACK_SEND, DATA_IN)
begin
if(rising_edge(Clk)) then
    DATA_RECEIVED <= TO_UNSIGNED(COUNTER_BYTE,8);
    case STATE is
    
    when STATE_IDLE =>
        ACK_SIG_SEND <= '0';
        RECIEVED_DATA <= '0';
        COUNTER <= 0;
        COUNTER_BYTE <= 0;
        ADDRESS <= (others => '0');
        DATA_IN <= (others => '0');
        if(SDA_VALUE = '0' AND SCL_VALUE = '0') then
            NEXT_STATE <= STATE_ADDRESS;
        else 
            NEXT_STATE <= STATE_IDLE;
        end if;        
    
    when STATE_ADDRESS =>
        if(SCL_VALUE = '1' and COUNTER < 8 and RECIEVED_DATA = '0') then
            ADDRESS(7 downto 1) <= ADDRESS(6 downto 0);
            ADDRESS(0)          <= SDA_VALUE;
            RECIEVED_DATA       <= '1';
            COUNTER             <= COUNTER + 1;
            NEXT_STATE          <= STATE_ADDRESS;
    
        elsif(SCL_VALUE = '0') then
            if (COUNTER = 8 and ADDRESS = SLAVE_ADDRESS) then
                NEXT_STATE      <= STATE_ACK_ADDRESS;
            elsif(COUNTER = 8 and ADDRESS /= SLAVE_ADDRESS) then
                NEXT_STATE     <= STATE_STOP; 
            else
                NEXT_STATE      <= STATE_ADDRESS;
                RECIEVED_DATA   <= '0';
            end if;       
        end if;
        
      when STATE_ACK_ADDRESS =>
        if(SCL_VALUE = '0' and ACK_SIG_SEND = '0') then -- Low clock first time (Before Clock going high with ack) 
            NEXT_STATE     <= STATE_ACK_ADDRESS;
            I2C_OUT        <= '1';
            ACK_SEND       <= '0';
            
             
        elsif(SCL_VALUE= '1' and ACK_SIG_SEND = '0') then 
            ACK_SIG_SEND   <= '1'; 
            NEXT_STATE     <= STATE_ACK_ADDRESS;
             
        elsif(SCL_VALUE = '0' and ACK_SIG_SEND = '1') then --Low Clock after Clock was going high with ACK and SET ACK to 0 again    
            I2C_OUT        <= '0';
            NEXT_STATE     <= STATE_READ_BYTE; 
                
        else    
            NEXT_STATE <= STATE_ACK_ADDRESS;        
        end if;
       
     when STATE_READ_BYTE => 
        if(SCL_VALUE = '1' and COUNTER_BYTE < 8 and RECIEVED_DATA = '0') then
            DATA_IN(7 downto 1) <= DATA_IN(6 downto 0);
            DATA_IN(0)          <= SDA_VALUE;
            RECIEVED_DATA       <= '1';
            COUNTER_BYTE        <= COUNTER_BYTE + 1;
            NEXT_STATE          <= STATE_READ_BYTE;
              
        
        elsif(SCL_VALUE = '0') then        
            if (COUNTER_BYTE = 8) then
                NEXT_STATE      <= STATE_ACK_BYTE;
                I2C_OUT         <= '0';
            elsif(ACK_SIG_SEND = '1') then
                ACK_SIG_SEND    <= '0';   
            else
                NEXT_STATE      <= STATE_READ_BYTE;
                RECIEVED_DATA   <= '0';
            end if;         
        end if;
     
     when STATE_ACK_BYTE =>
        if(SCL_VALUE = '0' and ACK_SIG_SEND = '0') then -- Low clock first time (Before Clock going high with ack) 
            I2C_OUT        <= '1';
            ACK_SEND       <= '0';
            NEXT_STATE     <= STATE_ACK_BYTE;
  
        elsif(SCL_VALUE= '1' and ACK_SIG_SEND = '0') then 
            ACK_SIG_SEND   <= '1'; 
            NEXT_STATE     <= STATE_ACK_BYTE;
            
             
        elsif(SCL_VALUE = '0' and ACK_SIG_SEND = '1') then --Low Clock after Clock was going high with ACK and SET ACK to 0 again    
            I2C_OUT        <= '0';
            COUNTER_BYTE   <= 0;
            DATA_IN        <= (others => '0');
            NEXT_STATE     <= STATE_READ_BYTE; 
               
        else         
            NEXT_STATE <= STATE_ACK_BYTE;        
        end if;
        
           
     when STATE_STOP =>
        if(SCL_VALUE = '1' and ACK_SIG_SEND = '1') then
            I2C_OUT <= '0';
            ACK_SEND <= '0';   
        elsif(I2C_EN = '1') then 
            NEXT_STATE <= STATE_STOP; 
        else 
            NEXT_STATE <= STATE_IDLE; 
        end if;          
    end case;   
end if;
end process;

-----------------------------------------------------
-- 
-----------------------------------------------------

end Behavioral;