library verilog;
use verilog.vl_types.all;
entity hazard_detect is
    port(
        clk             : in     vl_logic;
        op_ex           : in     vl_logic_vector(5 downto 0);
        load_reg        : in     vl_logic_vector(4 downto 0);
        rs_id           : in     vl_logic_vector(4 downto 0);
        rt_id           : in     vl_logic_vector(4 downto 0);
        enable_pc       : out    vl_logic;
        enable_ifid     : out    vl_logic;
        flush_idex      : out    vl_logic
    );
end hazard_detect;
