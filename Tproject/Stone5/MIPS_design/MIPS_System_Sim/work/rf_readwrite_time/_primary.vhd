library verilog;
use verilog.vl_types.all;
entity rf_readwrite_time is
    port(
        writereg        : in     vl_logic_vector(4 downto 0);
        readreg         : in     vl_logic_vector(4 downto 0);
        result          : in     vl_logic_vector(31 downto 0);
        read            : in     vl_logic_vector(31 downto 0);
        realread        : out    vl_logic_vector(31 downto 0)
    );
end rf_readwrite_time;
