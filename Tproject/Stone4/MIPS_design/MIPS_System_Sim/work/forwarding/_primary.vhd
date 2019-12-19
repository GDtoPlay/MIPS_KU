library verilog;
use verilog.vl_types.all;
entity forwarding is
    port(
        regwrite_wb     : in     vl_logic;
        regwrite_mem    : in     vl_logic;
        rs              : in     vl_logic_vector(4 downto 0);
        rt              : in     vl_logic_vector(4 downto 0);
        writereg_mem    : in     vl_logic_vector(4 downto 0);
        writereg_wb     : in     vl_logic_vector(4 downto 0);
        foward_rs       : out    vl_logic_vector(1 downto 0);
        foward_rt       : out    vl_logic_vector(1 downto 0)
    );
end forwarding;
