from pathlib import Path

from skidl import ERC, Net, Part, Pin, SKIDL, TEMPLATE, generate_netlist
from skidl.pin import pin_types

channel_count = 1
output_path = Path(__file__).with_name("single_channel_prototype.net")


def make_template(name: str, ref_prefix: str, footprint: str, pins: list[Pin]) -> Part:
    return Part(
        name=name,
        tool=SKIDL,
        dest=TEMPLATE,
        ref_prefix=ref_prefix,
        footprint=footprint,
        pins=pins,
    )


def main() -> None:
    adc_channel = make_template(
        "ADS131M08_CH",
        "U",
        "Package_QFP:TQFP-32_7x7mm_P0.8mm",
        [
            Pin(num="1", name="AINP", func=pin_types.INPUT),
            Pin(num="2", name="AINN", func=pin_types.INPUT),
        ],
    )
    electrode_pair = make_template(
        "ELECTRODE_PAIR",
        "J",
        "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
        [
            Pin(num="1", name="REC", func=pin_types.PASSIVE),
            Pin(num="2", name="REF", func=pin_types.PASSIVE),
        ],
    )

    for channel in range(channel_count):
        adc = adc_channel(tag=f"adc_ch{channel}")
        connector = electrode_pair(tag=f"electrode_ch{channel}")
        ainp = Net(f"AIN{channel}P")
        ainn = Net(f"AIN{channel}N")
        ainp += adc["AINP"], connector["REC"]
        ainn += adc["AINN"], connector["REF"]

    ERC()
    generate_netlist(file_=str(output_path), do_backup=False)
    print(f"Wrote {output_path}")
    print("Each channel maps recording -> AINxP and shared reference -> AINxN.")


if __name__ == "__main__":
    main()
