library verilog;
use verilog.vl_types.all;
entity ppline_ifid is
    port(
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        enable          : in     vl_logic;
        pc_plus4_d      : in     vl_logic_vector(31 downto 0);
        instr_d         : in     vl_logic_vector(31 downto 0);
        pc_plus4        : out    vl_logic_vector(31 downto 0);
        instr           : out    vl_logic_vector(31 downto 0)
    );
end ppline_ifid;
