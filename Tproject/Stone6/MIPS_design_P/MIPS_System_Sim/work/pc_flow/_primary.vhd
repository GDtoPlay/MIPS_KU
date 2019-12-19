library verilog;
use verilog.vl_types.all;
entity pc_flow is
    port(
        pc_plus4        : in     vl_logic_vector(31 downto 0);
        instr           : in     vl_logic_vector(31 downto 0);
        pc_branch       : in     vl_logic_vector(31 downto 0);
        aluzero         : in     vl_logic;
        aluout          : in     vl_logic_vector(31 downto 0);
        pcsrc           : in     vl_logic;
        jump            : in     vl_logic;
        alucontrol      : in     vl_logic_vector(3 downto 0);
        pc_next         : out    vl_logic_vector(31 downto 0)
    );
end pc_flow;
