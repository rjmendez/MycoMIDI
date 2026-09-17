from __future__ import annotations

from skidl import Net

from ads131m08_skidl import (
    SINGLE_CHANNEL_NETLIST,
    connect_standard_support,
    make_ads131m08_template,
    make_capacitor_template,
    make_electrode_pair_template,
    make_power_header_template,
    make_spi_header_template,
    write_netlist,
)


def main() -> None:
    adc = make_ads131m08_template()(value="ADS131M08", tag="adc")
    electrode_pair = make_electrode_pair_template()(value="CH0_PAIR", tag="electrode_pair")
    spi_header = make_spi_header_template()(value="SPI_HOST", tag="spi_host")
    power_header = make_power_header_template()(value="PWR_CTRL", tag="pwr_ctrl")
    cap_template = make_capacitor_template()

    nets = connect_standard_support(
        adc,
        spi_header=spi_header,
        power_header=power_header,
        cap_template=cap_template,
    )

    nets["AIN0P"] = nets.get("AIN0P") or Net("AIN0P")
    nets["AIN0N"] = nets.get("AIN0N") or Net("AIN0N")
    nets["AIN0P"] += adc["AIN0P"], electrode_pair["REC"]
    nets["AIN0N"] += adc["AIN0N"], electrode_pair["REF"]

    output_path = write_netlist(SINGLE_CHANNEL_NETLIST)
    print(f"Wrote {output_path}")
    print("Single-channel prototype uses the real ADS131M08 pinout, plus shared SPI/power support.")
    print("Channel 0 maps recording -> AIN0P and passive reference -> AIN0N.")
    print("REFP net aliases the actual ADS131M08 REFIN pin; the return side is GND/REFN-equivalent.")


if __name__ == "__main__":
    main()
