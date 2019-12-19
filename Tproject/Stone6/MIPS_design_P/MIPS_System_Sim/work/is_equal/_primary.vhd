library verilog;
use verilog.vl_types.all;
entity is_equal is
    port(
        d0              : in     vl_logic_vector(31 downto 0);
        d1              : in     vl_logic_vector(31 downto 0);
        eq              : out    vl_logic
    );
end is_equal;
