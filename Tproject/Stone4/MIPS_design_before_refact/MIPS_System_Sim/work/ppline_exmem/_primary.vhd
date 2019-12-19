library verilog;
use verilog.vl_types.all;
entity ppline_exmem is
    port(
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        pc_plus4_d      : in     vl_logic_vector(31 downto 0);
        aluzero_d       : in     vl_logic;
        aluout_d        : in     vl_logic_vector(31 downto 0);
        rd2_d           : in     vl_logic_vector(31 downto 0);
        writereg_d      : in     vl_logic_vector(4 downto 0);
        instr_d         : in     vl_logic_vector(31 downto 0);
        memwrite_d      : in     vl_logic;
        jumpAndLink_d   : in     vl_logic;
        regwrite_d      : in     vl_logic;
        memtoreg_d      : in     vl_logic;
        pc_plus4        : out    vl_logic_vector(31 downto 0);
        aluzero         : out    vl_logic;
        aluout          : out    vl_logic_vector(31 downto 0);
        rd2             : out    vl_logic_vector(31 downto 0);
        writereg        : out    vl_logic_vector(4 downto 0);
        instr           : out    vl_logic_vector(31 downto 0);
        memwrite        : out    vl_logic;
        jumpAndLink     : out    vl_logic;
        regwrite        : out    vl_logic;
        memtoreg        : out    vl_logic
    );
end ppline_exmem;
