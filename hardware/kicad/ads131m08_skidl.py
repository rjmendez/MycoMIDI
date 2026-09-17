from __future__ import annotations

from pathlib import Path

from skidl import Net, Part, Pin, SKIDL, TEMPLATE, generate_netlist
from skidl.pin import pin_types

CHANNEL_COUNT = 8
KICAD_ROOT = Path(__file__).resolve().parent
ADC_BOARD_DIR = KICAD_ROOT / "adc_board"
SINGLE_CHANNEL_NETLIST = KICAD_ROOT / "single_channel_prototype.net"
ADC_BOARD_NETLIST = ADC_BOARD_DIR / "adc_board_8ch.net"
ADC_BOARD_PCB = ADC_BOARD_DIR / "adc_board_8ch.kicad_pcb"

# ADS131M08 TQFP-32 pinout from TI datasheet SBAS950B Rev. B,
# cross-checked against Figure 5-1 (32-pin TQFP top view) and Table 5-1.
ADS131M08_PIN_DEFS: list[tuple[str, str, object]] = [
    ("1", "AIN2P", pin_types.INPUT),
    ("2", "AIN2N", pin_types.INPUT),
    ("3", "AIN3N", pin_types.INPUT),
    ("4", "AIN3P", pin_types.INPUT),
    ("5", "AIN4P", pin_types.INPUT),
    ("6", "AIN4N", pin_types.INPUT),
    ("7", "AIN5N", pin_types.INPUT),
    ("8", "AIN5P", pin_types.INPUT),
    ("9", "AIN6P", pin_types.INPUT),
    ("10", "AIN6N", pin_types.INPUT),
    ("11", "AIN7N", pin_types.INPUT),
    ("12", "AIN7P", pin_types.INPUT),
    ("13", "AGND", pin_types.PWRIN),
    ("14", "REFIN", pin_types.INPUT),
    ("15", "AVDD", pin_types.PWRIN),
    ("16", "SYNC_RESET", pin_types.INPUT),
    ("17", "CS", pin_types.INPUT),
    ("18", "DRDY", pin_types.OUTPUT),
    ("19", "SCLK", pin_types.INPUT),
    ("20", "DOUT", pin_types.OUTPUT),
    ("21", "DIN", pin_types.INPUT),
    ("22", "XTAL2", pin_types.OUTPUT),
    ("23", "CLKIN", pin_types.INPUT),
    ("24", "CAP", pin_types.PASSIVE),
    ("25", "DGND", pin_types.PWRIN),
    ("26", "DVDD", pin_types.PWRIN),
    ("27", "NC", pin_types.NOCONNECT),
    ("28", "AGND_2", pin_types.PWRIN),
    ("29", "AIN0P", pin_types.INPUT),
    ("30", "AIN0N", pin_types.INPUT),
    ("31", "AIN1N", pin_types.INPUT),
    ("32", "AIN1P", pin_types.INPUT),
]


def make_template(name: str, ref_prefix: str, footprint: str, pins: list[Pin]) -> Part:
    return Part(
        name=name,
        tool=SKIDL,
        dest=TEMPLATE,
        ref_prefix=ref_prefix,
        footprint=footprint,
        pins=pins,
    )


def make_ads131m08_template() -> Part:
    return make_template(
        "ADS131M08",
        "U",
        "Package_QFP:TQFP-32_7x7mm_P0.8mm",
        [Pin(num=num, name=name, func=func) for num, name, func in ADS131M08_PIN_DEFS],
    )


def make_electrode_pair_template() -> Part:
    return make_template(
        "ELECTRODE_PAIR",
        "J",
        "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
        [
            Pin(num="1", name="REC", func=pin_types.PASSIVE),
            Pin(num="2", name="REF", func=pin_types.PASSIVE),
        ],
    )


def make_electrode_header_template() -> Part:
    pins: list[Pin] = []
    for channel in range(CHANNEL_COUNT):
        base_pin = (channel * 2) + 1
        pins.extend(
            [
                Pin(num=str(base_pin), name=f"CH{channel}_REC", func=pin_types.PASSIVE),
                Pin(num=str(base_pin + 1), name=f"CH{channel}_REF", func=pin_types.PASSIVE),
            ]
        )
    return make_template(
        "CONN_02X08_ELECTRODE_HEADER",
        "J",
        "Connector_PinHeader_2.54mm:PinHeader_2x08_P2.54mm_Vertical",
        pins,
    )


def make_spi_header_template() -> Part:
    return make_template(
        "SPI_HOST_HEADER",
        "J",
        "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical",
        [
            Pin(num="1", name="CS", func=pin_types.PASSIVE),
            Pin(num="2", name="SCLK", func=pin_types.PASSIVE),
            Pin(num="3", name="DIN", func=pin_types.PASSIVE),
            Pin(num="4", name="DOUT", func=pin_types.PASSIVE),
            Pin(num="5", name="DRDY", func=pin_types.PASSIVE),
        ],
    )


def make_power_header_template() -> Part:
    return make_template(
        "POWER_CTRL_HEADER",
        "J",
        "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical",
        [
            Pin(num="1", name="AVDD", func=pin_types.PWROUT),
            Pin(num="2", name="DVDD", func=pin_types.PWROUT),
            Pin(num="3", name="GND", func=pin_types.PWROUT),
            Pin(num="4", name="CLKIN", func=pin_types.PASSIVE),
            Pin(num="5", name="SYNC_RESET", func=pin_types.PASSIVE),
        ],
    )


def make_capacitor_template() -> Part:
    return make_template(
        "C",
        "C",
        "Capacitor_SMD:C_0603_1608Metric",
        [
            Pin(num="1", name="1", func=pin_types.PASSIVE),
            Pin(num="2", name="2", func=pin_types.PASSIVE),
        ],
    )


def connect_standard_support(
    adc: Part,
    *,
    spi_header: Part,
    power_header: Part,
    cap_template: Part,
) -> dict[str, Net]:
    nets = {
        "AVDD": Net("AVDD"),
        "DVDD": Net("DVDD"),
        "GND": Net("GND"),
        "CLKIN": Net("CLKIN"),
        "SYNC_RESET": Net("SYNC_RESET"),
        "CS": Net("CS"),
        "SCLK": Net("SCLK"),
        "DIN": Net("DIN"),
        "DOUT": Net("DOUT"),
        "DRDY": Net("DRDY"),
        # The ADS131M08 TQFP exposes REFIN and CAP, not separate REFP/REFN pins.
        # This prototype names the positive reference node REFP for readability and
        # uses the common ground return as the effective REFN side.
        "REFP": Net("REFP"),
        "CAP": Net("CAP"),
    }

    nets["AVDD"] += adc["AVDD"], power_header["AVDD"]
    nets["DVDD"] += adc["DVDD"], power_header["DVDD"]
    nets["GND"] += adc["AGND"], adc["AGND_2"], adc["DGND"], power_header["GND"]
    nets["CLKIN"] += adc["CLKIN"], power_header["CLKIN"]
    nets["SYNC_RESET"] += adc["SYNC_RESET"], power_header["SYNC_RESET"]
    nets["CS"] += adc["CS"], spi_header["CS"]
    nets["SCLK"] += adc["SCLK"], spi_header["SCLK"]
    nets["DIN"] += adc["DIN"], spi_header["DIN"]
    nets["DOUT"] += adc["DOUT"], spi_header["DOUT"]
    nets["DRDY"] += adc["DRDY"], spi_header["DRDY"]
    nets["REFP"] += adc["REFIN"]
    nets["CAP"] += adc["CAP"]

    c_avdd_bypass = cap_template(value="100n", tag="avdd_bypass")
    c_avdd_bulk = cap_template(value="1u", tag="avdd_bulk")
    c_dvdd_bypass = cap_template(value="100n", tag="dvdd_bypass")
    c_dvdd_bulk = cap_template(value="1u", tag="dvdd_bulk")
    c_ref = cap_template(value="1u", tag="refin_bulk")
    c_cap = cap_template(value="220n", tag="cap_ldo")

    for capacitor in (c_avdd_bypass, c_avdd_bulk):
        nets["AVDD"] += capacitor["1"]
        nets["GND"] += capacitor["2"]

    for capacitor in (c_dvdd_bypass, c_dvdd_bulk):
        nets["DVDD"] += capacitor["1"]
        nets["GND"] += capacitor["2"]

    nets["REFP"] += c_ref["1"]
    nets["GND"] += c_ref["2"]
    nets["CAP"] += c_cap["1"]
    nets["GND"] += c_cap["2"]

    return nets


def write_netlist(output_path: Path) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    generate_netlist(file_=str(output_path), do_backup=False)
    return output_path
