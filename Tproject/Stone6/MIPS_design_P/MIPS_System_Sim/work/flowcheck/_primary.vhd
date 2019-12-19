library verilog;
use verilog.vl_types.all;
entity flowcheck is
    port(
        alucontrol      : in     vl_logic_vector(3 downto 0);
        branch          : in     vl_logic;
        branchN         : in     vl_logic;
        jump            : in     vl_logic;
        aluzero         : in     vl_logic;
        flush           : out    vl_logic
    );
end flowcheck;
