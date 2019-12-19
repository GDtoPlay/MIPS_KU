library verilog;
use verilog.vl_types.all;
entity id_part is
    port(
        clk             : in     vl_logic;
        instr           : in     vl_logic_vector(31 downto 0);
        writereg        : in     vl_logic_vector(4 downto 0);
        result          : in     vl_logic_vector(31 downto 0);
        pc_plus4        : in     vl_logic_vector(31 downto 0);
        readreg_1       : out    vl_logic_vector(31 downto 0);
        readreg_2       : out    vl_logic_vector(31 downto 0);
        signimm         : out    vl_logic_vector(31 downto 0);
        rs              : out    vl_logic_vector(4 downto 0);
        rd              : out    vl_logic_vector(4 downto 0);
        rt              : out    vl_logic_vector(4 downto 0);
        shiftl16        : out    vl_logic;
        regdst          : out    vl_logic;
        alusrc          : out    vl_logic;
        branch          : out    vl_logic;
        branchN         : out    vl_logic;
        jump            : out    vl_logic;
        memwrite        : out    vl_logic;
        jumpAndLink     : out    vl_logic;
        memtoreg        : out    vl_logic;
        regwrite        : out    vl_logic;
        alucontrol      : out    vl_logic_vector(3 downto 0);
        regwrite_wb     : in     vl_logic
    );
end id_part;
