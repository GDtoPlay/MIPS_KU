library verilog;
use verilog.vl_types.all;
entity ppline_memwb is
    port(
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        pc_plus4_d      : in     vl_logic_vector(31 downto 0);
        readdata_d      : in     vl_logic_vector(31 downto 0);
        aluout_d        : in     vl_logic_vector(31 downto 0);
        writereg_d      : in     vl_logic_vector(4 downto 0);
        jumpAndLink_d   : in     vl_logic;
        regwrite_d      : in     vl_logic;
        memtoreg_d      : in     vl_logic;
        pc_plus4        : out    vl_logic_vector(31 downto 0);
        readdata        : out    vl_logic_vector(31 downto 0);
        aluout          : out    vl_logic_vector(31 downto 0);
        writereg        : out    vl_logic_vector(4 downto 0);
        jumpAndLink     : out    vl_logic;
        regwrite        : out    vl_logic;
        memtoreg        : out    vl_logic
    );
end ppline_memwb;
