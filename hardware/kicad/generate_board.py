from pathlib import Path

from skidl import ERC, Net, Part, Pin, SKIDL, TEMPLATE, generate_netlist
from skidl.pin import pin_types

CHANNEL_COUNT = 8
OUTPUT_PATH = Path(__file__).with_name("electrode_board_8ch.net")


def make_template(name: str, ref_prefix: str, footprint: str, pins: list[Pin]) -> Part:
    return Part(
        name=name,
        tool=SKIDL,
        dest=TEMPLATE,
        ref_prefix=ref_prefix,
        footprint=footprint,
        pins=pins,
    )


def make_adc_module_template() -> Part:
    pins: list[Pin] = []
    for channel in range(CHANNEL_COUNT):
        base_pin = (channel * 2) + 1
        pins.extend(
            [
                Pin(num=str(base_pin), name=f"AIN{channel}P", func=pin_types.INPUT),
                Pin(num=str(base_pin + 1), name=f"AIN{channel}N", func=pin_types.INPUT),
            ]
        )
    return make_template(
        "ADS131M08_MODULE",
        "U",
        "Package_QFP:TQFP-32_7x7mm_P0.8mm",
        pins,
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


def make_test_point_template() -> Part:
    return make_template(
        "TESTPOINT_1P",
        "TP",
        "TestPoint:TestPoint_Pad_D1.0mm",
        [Pin(num="1", name="TP", func=pin_types.PASSIVE)],
    )


def main() -> None:
    adc = make_adc_module_template()(tag="adc_module")
    header = make_electrode_header_template()(tag="electrode_header")
    bias_stub = make_test_point_template()(tag="bias_drive_placeholder")

    for channel in range(CHANNEL_COUNT):
        ainp = Net(f"AIN{channel}P")
        ainn = Net(f"AIN{channel}N")
        ainp += adc[f"AIN{channel}P"], header[f"CH{channel}_REC"]
        ainn += adc[f"AIN{channel}N"], header[f"CH{channel}_REF"]

    bias_drive_todo = Net("BIAS_DRIVE_TODO")
    bias_drive_todo += bias_stub["TP"]

    ERC()
    generate_netlist(file_=str(OUTPUT_PATH), do_backup=False)
    print(f"Wrote {OUTPUT_PATH}")
    print("Mapped recording electrodes to AINxP and reference electrodes to AINxN for 8 channels.")
    print("All 8 channel pairs land on one 16-pin CONN_02X08-style header footprint.")
    print("Bias-drive circuit remains a TODO placeholder on net BIAS_DRIVE_TODO -> TP1.")


if __name__ == "__main__":
    main()
