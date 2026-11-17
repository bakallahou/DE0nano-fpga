-- ============================================================================
--  drone_top.vhd  -  Projet DE0-Nano : ADXL345 -> UART -> ESP32 -> MATLAB
--
--  Sortie UART : UART_TX = GPIO_00 = PIN_D3 = connecteur JP1, broche 2
--                115200 bauds, 8N1, 3.3 V, trames ASCII "ax,ay,az\n" (LSB)
--  LEDs : LED0 init OK | LED1 ID ADXL345 OK (0xE5) | LED2 clignote = trames
--         LED3 battement | LED4..7 barre d'inclinaison (axe Y)
--  KEY0 : reset (actif a l'etat bas)
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity drone_top is
  generic (
    SAMPLE_HZ : integer := 50;
    -- Reglages d'orientation (si l'avion tourne dans le mauvais sens) :
    SWAP_XY   : boolean := false;
    INVERT_X  : boolean := false;
    INVERT_Y  : boolean := false;
    INVERT_Z  : boolean := false
  );
  port (
    CLOCK_50      : in    std_logic;
    KEY           : in    std_logic_vector(1 downto 0);
    LED           : out   std_logic_vector(7 downto 0);
    G_SENSOR_CS_N : out   std_logic;
    G_SENSOR_INT  : in    std_logic;
    I2C_SCLK      : out   std_logic;
    I2C_SDAT      : inout std_logic;
    UART_TX       : out   std_logic
  );
end entity;

architecture rtl of drone_top is
  signal rst_n, k1, k2        : std_logic := '0';
  signal ax, ay, az           : signed(15 downto 0);
  signal sx0, sy0             : signed(15 downto 0);
  signal sx, sy, sz           : signed(15 downto 0);
  signal s_valid, s_init, s_id: std_logic;
  signal u_data               : std_logic_vector(7 downto 0);
  signal u_valid, u_busy      : std_logic;
  signal f_busy               : std_logic;
  signal frame_cnt            : unsigned(7 downto 0) := (others => '0');
  signal hb                   : unsigned(25 downto 0) := (others => '0');
begin
  -- Synchronisation du bouton de reset
  process(CLOCK_50)
  begin
    if rising_edge(CLOCK_50) then
      k1 <= KEY(0); k2 <= k1; rst_n <= k2;
    end if;
  end process;

  -- Accelerometre (SPI 3 fils)
  u_adxl : entity work.adxl345_spi
    generic map (CLK_FREQ => 50_000_000, SPI_FREQ => 1_000_000, SAMPLE_HZ => SAMPLE_HZ)
    port map (clk => CLOCK_50, rst_n => rst_n,
              cs_n => G_SENSOR_CS_N, sclk => I2C_SCLK, sdio => I2C_SDAT,
              ax => ax, ay => ay, az => az,
              valid => s_valid, init_done => s_init, id_ok => s_id);

  -- Remise en repere (echange / inversion d'axes)
  sx0 <= ay when SWAP_XY else ax;
  sy0 <= ax when SWAP_XY else ay;
  sx  <= -sx0 when INVERT_X else sx0;
  sy  <= -sy0 when INVERT_Y else sy0;
  sz  <= -az  when INVERT_Z else az;

  -- Formatage ASCII "ax,ay,az\n"
  u_frame : entity work.frame_tx
    port map (clk => CLOCK_50, rst_n => rst_n, start => s_valid,
              x => sx, y => sy, z => sz,
              tx_busy => u_busy, tx_data => u_data, tx_valid => u_valid, busy => f_busy);

  -- UART 115200 8N1
  u_uart : entity work.uart_tx
    generic map (CLK_FREQ => 50_000_000, BAUD => 115_200)
    port map (clk => CLOCK_50, rst_n => rst_n,
              data => u_data, valid => u_valid, busy => u_busy, tx => UART_TX);

  -- Voyants
  process(CLOCK_50)
  begin
    if rising_edge(CLOCK_50) then
      hb <= hb + 1;
      if s_valid = '1' then
        frame_cnt <= frame_cnt + 1;
      end if;
    end if;
  end process;

  LED(0) <= s_init;
  LED(1) <= s_id;
  LED(2) <= frame_cnt(4);                 -- clignote ~3 Hz si les trames partent
  LED(3) <= hb(25);                       -- battement ~0.75 Hz
  LED(4) <= '1' when sy < -128 else '0';  -- inclinaison forte d'un cote
  LED(5) <= '1' when sy < -32  else '0';
  LED(6) <= '1' when sy >  32  else '0';
  LED(7) <= '1' when sy >  128 else '0';  -- inclinaison forte de l'autre
end architecture;
