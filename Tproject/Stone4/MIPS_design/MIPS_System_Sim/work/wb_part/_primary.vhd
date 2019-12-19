library verilog;
use verilog.vl_types.all;
entity wb_part is
    port(
        pc_plus4        : in     vl_logic_vector(31 downto 0);
        aluout          : in     vl_logic_vector(31 downto 0);
        readdata        : in     vl_logic_vector(31 downto 0);
        jumpAndLink     : in     vl_logic;
        memtoreg        : in     vl_logic;
        result          : out    vl_logic_vector(31 downto 0)
    );
end wb_part;
