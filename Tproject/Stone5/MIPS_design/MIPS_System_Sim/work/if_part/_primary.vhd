library verilog;
use verilog.vl_types.all;
entity if_part is
    port(
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        enable          : in     vl_logic;
        pc_next         : in     vl_logic_vector(31 downto 0);
        pc              : out    vl_logic_vector(31 downto 0);
        pc_plus4        : out    vl_logic_vector(31 downto 0)
    );
end if_part;
