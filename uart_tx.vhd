-- ============================================================================
--  uart_tx.vhd  -  Emetteur UART 8N1
--  Interface : valid = impulsion 1 cycle (quand busy='0') avec data stable.
--  busy passe a '1' au cycle suivant et retombe a la fin du bit de stop.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity uart_tx is
  generic (
    CLK_FREQ : integer := 50_000_000;
    BAUD     : integer := 115_200
  );
  port (
    clk   : in  std_logic;
    rst_n : in  std_logic;
    data  : in  std_logic_vector(7 downto 0);
    valid : in  std_logic;
    busy  : out std_logic;
    tx    : out std_logic
  );
end entity;

architecture rtl of uart_tx is
  constant DIV : integer := CLK_FREQ / BAUD;          -- 434 coups d'horloge par bit
  type st_t is (IDLE, START_B, DATA_B, STOP_B);
  signal st    : st_t := IDLE;
  signal cnt   : integer range 0 to DIV-1 := 0;
  signal bitn  : integer range 0 to 7 := 0;
  signal sh    : std_logic_vector(7 downto 0) := (others => '0');
  signal tx_r  : std_logic := '1';
begin
  tx   <= tx_r;
  busy <= '0' when st = IDLE else '1';

  process(clk)
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        st <= IDLE; tx_r <= '1'; cnt <= 0; bitn <= 0;
      else
        case st is
          when IDLE =>
            tx_r <= '1'; cnt <= 0; bitn <= 0;
            if valid = '1' then
              sh   <= data;
              tx_r <= '0';                     -- bit de start
              st   <= START_B;
            end if;

          when START_B =>
            if cnt = DIV-1 then
              cnt  <= 0;
              tx_r <= sh(0);
              st   <= DATA_B;
            else
              cnt <= cnt + 1;
            end if;

          when DATA_B =>
            if cnt = DIV-1 then
              cnt <= 0;
              if bitn = 7 then
                tx_r <= '1';                   -- bit de stop
                st   <= STOP_B;
              else
                bitn <= bitn + 1;
                tx_r <= sh(bitn + 1);
              end if;
            else
              cnt <= cnt + 1;
            end if;

          when STOP_B =>
            if cnt = DIV-1 then
              cnt <= 0;
              st  <= IDLE;
            else
              cnt <= cnt + 1;
            end if;
        end case;
      end if;
    end if;
  end process;
end architecture;
