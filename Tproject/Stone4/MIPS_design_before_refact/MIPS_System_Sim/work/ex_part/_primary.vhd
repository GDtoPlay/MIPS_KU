library verilog;
use verilog.vl_types.all;
entity ex_part is
    port(
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        readreg_1       : in     vl_logic_vector(31 downto 0);
        readreg_2       : in     vl_logic_vector(31 downto 0);
        foward_mem      : in     vl_logic_vector(31 downto 0);
        foward_wd       : in     vl_logic_vector(31 downto 0);
        foward_rs       : in     vl_logic_vector(1 downto 0);
        foward_rt       : in     vl_logic_vector(1 downto 0);
        signimm         : in     vl_logic_vector(31 downto 0);
        instr           : in     vl_logic_vector(31 downto 0);
        rs              : in     vl_logic_vector(4 downto 0);
        rt              : in     vl_logic_vector(4 downto 0);
        rd              : in     vl_logic_vector(4 downto 0);
        shiftl16        : in     vl_logic;
        regdst          : in     vl_logic;
        alucontrol      : in     vl_logic_vector(3 downto 0);
        alusrc          : in     vl_logic;
        jumpAndLink     : in     vl_logic;
        aluzero         : out    vl_logic;
        aluout          : out    vl_logic_vector(31 downto 0);
        writereg        : out    vl_logic_vector(4 downto 0);
        write_data      : out    vl_logic_vector(31 downto 0)
    );
end ex_part;
