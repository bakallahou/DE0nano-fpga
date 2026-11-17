-- ============================================================================
--  frame_tx.vhd  -  Formate "ax,ay,az\n" en ASCII decimal et l'envoie a l'UART
--  Valeurs signees en LSB (256 LSB = 1 g). Exemple : "-12,5,254\n"
--  Aucune division : conversion par soustractions successives des puissances de 10.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity frame_tx is
  port (
    clk      : in  std_logic;
    rst_n    : in  std_logic;
    start    : in  std_logic;                       -- impulsion : envoyer une trame
    x, y, z  : in  signed(15 downto 0);
    tx_busy  : in  std_logic;
    tx_data  : out std_logic_vector(7 downto 0);
    tx_valid : out std_logic;
    busy     : out std_logic
  );
end entity;

architecture rtl of frame_tx is
  type st_t is (S_IDLE, S_SIGN, S_DIG, S_DIGEND, S_SEP, S_SEND, S_W1, S_W2);
  type int3_t is array (0 to 2) of integer range -32768 to 32767;
  type pow_t  is array (0 to 4) of integer range 1 to 10000;
  constant POW : pow_t := (10000, 1000, 100, 10, 1);

  signal st, after_st : st_t := S_IDLE;
  signal vals    : int3_t := (others => 0);
  signal fld     : integer range 0 to 2 := 0;
  signal v       : integer range 0 to 32768 := 0;
  signal p       : integer range 0 to 4 := 0;
  signal dig     : integer range 0 to 9 := 0;
  signal started : std_logic := '0';
  signal chr     : std_logic_vector(7 downto 0) := (others => '0');
  signal valid_r : std_logic := '0';
begin
  tx_data  <= chr;
  tx_valid <= valid_r;
  busy     <= '0' when st = S_IDLE else '1';

  process(clk)
  begin
    if rising_edge(clk) then
      valid_r <= '0';
      if rst_n = '0' then
        st <= S_IDLE; after_st <= S_IDLE;
      else
        case st is
          when S_IDLE =>
            if start = '1' then
              vals(0) <= to_integer(x);
              vals(1) <= to_integer(y);
              vals(2) <= to_integer(z);
              fld <= 0;
              st  <= S_SIGN;
            end if;

          when S_SIGN =>
            p <= 0; dig <= 0; started <= '0';
            if vals(fld) < 0 then
              v <= -vals(fld);
              chr <= x"2D";                          -- '-'
              after_st <= S_DIG;
              st <= S_SEND;
            else
              v <= vals(fld);
              st <= S_DIG;
            end if;

          when S_DIG =>
            if v >= POW(p) then
              v   <= v - POW(p);
              dig <= dig + 1;
            else
              if dig > 0 or started = '1' or p = 4 then
                chr <= std_logic_vector(to_unsigned(48 + dig, 8));
                started <= '1';
                after_st <= S_DIGEND;
                st <= S_SEND;
              else
                st <= S_DIGEND;                      -- zero non significatif : on saute
              end if;
            end if;

          when S_DIGEND =>
            if p = 4 then
              st <= S_SEP;
            else
              p <= p + 1; dig <= 0;
              st <= S_DIG;
            end if;

          when S_SEP =>
            if fld = 2 then
              chr <= x"0A";                          -- '\n'
              after_st <= S_IDLE;
            else
              chr <= x"2C";                          -- ','
              fld <= fld + 1;
              after_st <= S_SIGN;
            end if;
            st <= S_SEND;

          when S_SEND =>
            if tx_busy = '0' then
              valid_r <= '1';
              st <= S_W1;
            end if;

          when S_W1 =>
            st <= S_W2;

          when S_W2 =>
            if tx_busy = '0' then
              st <= after_st;
            end if;
        end case;
      end if;
    end if;
  end process;
end architecture;
