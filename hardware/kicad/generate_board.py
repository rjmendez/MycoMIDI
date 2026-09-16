from __future__ import annotations

from skidl import Net

from ads131m08_skidl import (
    ADC_BOARD_NETLIST,
    CHANNEL_COUNT,
    connect_standard_support,
    make_ads131m08_template,
    make_capacitor_template,
    make_electrode_header_template,
    make_power_header_template,
    make_spi_header_template,
    write_netlist,
)


def main() -> None:
    adc = make_ads131m08_template()(value="ADS131M08", tag="adc")
    electrode_header = make_electrode_header_template()(value="ELECTRODES", tag="electrodes")
    spi_header = make_spi_header_template()(value="SPI_HOST", tag="spi_host")
    power_header = make_power_header_template()(value="PWR_CTRL", tag="pwr_ctrl")
    cap_template = make_capacitor_template()

    nets = connect_standard_support(
        adc,
        spi_header=spi_header,
        power_header=power_header,
        cap_template=cap_template,
    )

    for channel in range(CHANNEL_COUNT):
        ainp = Net(f"AIN{channel}P")
        ainn = Net(f"AIN{channel}N")
        ainp += adc[f"AIN{channel}P"], electrode_header[f"CH{channel}_REC"]
        ainn += adc[f"AIN{channel}N"], electrode_header[f"CH{channel}_REF"]
        nets[ainp.name] = ainp
        nets[ainn.name] = ainn

    output_path = write_netlist(ADC_BOARD_NETLIST)
    print(f"Wrote {output_path}")
    print("8-channel board netlist uses one real ADS131M08 TQFP-32 with exact Table 5-1 pin numbers.")
    print("Electrode header maps CHx_REC -> AINxP and CHx_REF -> AINxN for channels 0..7.")
    print("Support wiring includes SPI, CLKIN, SYNC_RESET, AVDD/DVDD bulk+bypass caps, REFIN cap, and CAP LDO cap.")
    print("REFP net aliases the ADS131M08 REFIN pin; the return side is the common GND node.")


if __name__ == "__main__":
    main()
