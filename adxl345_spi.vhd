-- ============================================================================
--  adxl345_spi.vhd  -  Lecture de l'accelerometre ADXL345 de la DE0-Nano
--
--  Liaison : SPI 3 fils (SDIO bidirectionnel), mode 3 (CPOL=1, CPHA=1).
--  Broches DE0-Nano : G_SENSOR_CS_N (PIN_G5), I2C_SCLK (PIN_F2, utilise comme
--  SCLK SPI), I2C_SDAT (PIN_F1, utilise comme SDIO).
--
--  Sequence d'initialisation :
--    0) DATA_FORMAT (0x31) = 0x48 : SPI 3 fils, pleine resolution, +-2 g
--    1) lecture DEVID (0x00)      : doit valoir 0xE5  -> id_ok
--    2) BW_RATE (0x2C)     = 0x0A : 100 Hz
--    3) POWER_CTL (0x2D)   = 0x08 : mode mesure
--  Puis lecture en rafale de 6 octets (0x32..0x37) toutes les 1/SAMPLE_HZ s.
--  Pleine resolution : 256 LSB par g (3.9 mg/LSB), valeurs signees.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity adxl345_spi is
  generic (
    CLK_FREQ  : integer := 50_000_000;
    SPI_FREQ  : integer := 1_000_000;     -- max 5 MHz pour l'ADXL345
    SAMPLE_HZ : integer := 50
  );
  port (
    clk       : in    std_logic;
    rst_n     : in    std_logic;
    cs_n      : out   std_logic;
    sclk      : out   std_logic;
    sdio      : inout std_logic;
    ax        : out   signed(15 downto 0);
    ay        : out   signed(15 downto 0);
    az        : out   signed(15 downto 0);
    valid     : out   std_logic;          -- impulsion : nouvel echantillon
    init_done : out   std_logic;
    id_ok     : out   std_logic
  );
end entity;

architecture rtl of adxl345_spi is
  constant HALF        : integer := CLK_FREQ / (2 * SPI_FREQ);   -- demi-periode SCLK
  constant PWRUP_CLKS  : integer := CLK_FREQ / 100;              -- 10 ms
  constant SAMPLE_CLKS : integer := CLK_FREQ / SAMPLE_HZ;

  type st_t is (ST_PWRUP, ST_LOAD, ST_CSSET, ST_LOW, ST_HIGH, ST_CSEND, ST_CSHI);
  signal st       : st_t := ST_PWRUP;
  signal step     : integer range 0 to 4 := 0;
  signal half_cnt : integer range 0 to HALF := 0;
  signal tmr      : integer range 0 to 16777215 := 0;
  signal smp_tmr  : integer range 0 to 16777215 := 0;
  signal bit_idx  : integer range 0 to 63 := 0;
  signal out_bits : integer range 0 to 63 := 0;
  signal in_bits  : integer range 0 to 63 := 0;
  signal sh_out   : std_logic_vector(15 downto 0) := (others => '0');
  signal sh_in    : std_logic_vector(47 downto 0) := (others => '0');
  signal sdio_o   : std_logic := '0';
  signal sdio_oe  : std_logic := '0';
  signal cs_r     : std_logic := '1';
  signal sclk_r   : std_logic := '1';
  signal init_r   : std_logic := '0';
  signal id_r     : std_logic := '0';
  signal valid_r  : std_logic := '0';
  signal ax_r, ay_r, az_r : signed(15 downto 0) := (others => '0');
begin
  cs_n      <= cs_r;
  sclk      <= sclk_r;
  sdio      <= sdio_o when sdio_oe = '1' else 'Z';
  ax        <= ax_r;
  ay        <= ay_r;
  az        <= az_r;
  valid     <= valid_r;
  init_done <= init_r;
  id_ok     <= id_r;

  process(clk)
    variable nxt : st_t;
  begin
    if rising_edge(clk) then
      valid_r <= '0';

      if rst_n = '0' then
        st <= ST_PWRUP; step <= 0; tmr <= 0; smp_tmr <= 0; half_cnt <= 0;
        cs_r <= '1'; sclk_r <= '1'; sdio_oe <= '0';
        init_r <= '0'; id_r <= '0';
      else
        if smp_tmr < SAMPLE_CLKS then
          smp_tmr <= smp_tmr + 1;
        end if;

        case st is
          -- Attente de mise sous tension du capteur (10 ms)
          when ST_PWRUP =>
            cs_r <= '1'; sclk_r <= '1'; sdio_oe <= '0';
            if tmr = PWRUP_CLKS then
              tmr <= 0; st <= ST_LOAD;
            else
              tmr <= tmr + 1;
            end if;

          -- Choix de la transaction suivante
          when ST_LOAD =>
            if step = 4 and smp_tmr < SAMPLE_CLKS then
              null;                                   -- attendre le prochain echantillon
            else
              case step is
                when 0      => sh_out <= x"3148"; out_bits <= 16; in_bits <= 0;
                when 1      => sh_out <= x"8000"; out_bits <= 8;  in_bits <= 8;
                when 2      => sh_out <= x"2C0A"; out_bits <= 16; in_bits <= 0;
                when 3      => sh_out <= x"2D08"; out_bits <= 16; in_bits <= 0;
                when others => sh_out <= x"F200"; out_bits <= 8;  in_bits <= 48;
                               smp_tmr <= 0;
              end case;
              bit_idx  <= 0;
              half_cnt <= 0;
              cs_r     <= '0';
              st       <= ST_CSSET;
            end if;

          -- Delai CS bas -> premier front d'horloge
          when ST_CSSET =>
            if half_cnt = HALF-1 then
              half_cnt <= 0; st <= ST_LOW;
            else
              half_cnt <= half_cnt + 1;
            end if;

          -- SCLK bas : le maitre place le bit suivant
          when ST_LOW =>
            sclk_r <= '0';
            if half_cnt = 0 then
              if bit_idx < out_bits then
                sdio_oe <= '1';
                sdio_o  <= sh_out(15);
                sh_out  <= sh_out(14 downto 0) & '0';
              else
                sdio_oe <= '0';                       -- le capteur pilote SDIO
              end if;
            end if;
            if half_cnt = HALF-1 then
              half_cnt <= 0; st <= ST_HIGH;
            else
              half_cnt <= half_cnt + 1;
            end if;

          -- SCLK haut : le capteur echantillonne / le maitre echantillonne
          when ST_HIGH =>
            sclk_r <= '1';
            if half_cnt = 3 and bit_idx = out_bits - 1 and in_bits > 0 then
              sdio_oe <= '0';                         -- liberer la ligne avant la lecture
            end if;
            if half_cnt = HALF-2 and bit_idx >= out_bits then
              sh_in <= sh_in(46 downto 0) & to_x01(sdio);
            end if;
            if half_cnt = HALF-1 then
              half_cnt <= 0;
              if bit_idx = out_bits + in_bits - 1 then
                st <= ST_CSEND;
              else
                bit_idx <= bit_idx + 1;
                st <= ST_LOW;
              end if;
            else
              half_cnt <= half_cnt + 1;
            end if;

          -- Fin de trame : CS repasse a '1'
          when ST_CSEND =>
            sdio_oe <= '0';
            if half_cnt = HALF-1 then
              half_cnt <= 0; cs_r <= '1'; st <= ST_CSHI;
            else
              half_cnt <= half_cnt + 1;
            end if;

          -- CS haut, puis exploitation du resultat
          when ST_CSHI =>
            if half_cnt = HALF-1 then
              half_cnt <= 0;
              nxt := ST_LOAD;
              case step is
                when 0 => step <= 1;
                when 1 =>
                  if sh_in(7 downto 0) = x"E5" then
                    id_r <= '1'; step <= 2;
                  else
                    id_r <= '0'; step <= 0; nxt := ST_PWRUP;   -- capteur muet : on recommence
                  end if;
                when 2 => step <= 3;
                when 3 => step <= 4; init_r <= '1';
                when others =>
                  ax_r <= signed(sh_in(39 downto 32) & sh_in(47 downto 40));
                  ay_r <= signed(sh_in(23 downto 16) & sh_in(31 downto 24));
                  az_r <= signed(sh_in(7 downto 0)   & sh_in(15 downto 8));
                  valid_r <= '1';
              end case;
              st <= nxt;
            else
              half_cnt <= half_cnt + 1;
            end if;
        end case;
      end if;
    end if;
  end process;
end architecture;
