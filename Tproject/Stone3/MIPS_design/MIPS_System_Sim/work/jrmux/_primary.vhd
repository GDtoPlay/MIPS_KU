library verilog;
use verilog.vl_types.all;
entity jrmux is
    port(
        d0              : in     vl_logic_vector(31 downto 0);
        d1              : in     vl_logic_vector(31 downto 0);
        alucontrol      : in     vl_logic_vector(3 downto 0);
        y               : out    vl_logic_vector(31 downto 0)
    );
end jrmux;
