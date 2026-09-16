from pathlib import Path

from skidl import ERC, Net, Part, Pin, SKIDL, TEMPLATE, generate_netlist
from skidl.pin import pin_types

OUTPUT_PATH = Path(__file__).with_name("active_probe_flex.net")


def make_template(name: str, ref_prefix: str, footprint: str, pins: list[Pin]) -> Part:
    return Part(
        name=name,
        tool=SKIDL,
        dest=TEMPLATE,
        ref_prefix=ref_prefix,
        footprint=footprint,
        pins=pins,
    )


def make_resistor_template() -> Part:
    return make_template(
        "RESISTOR",
        "R",
        "Resistor_SMD:R_0603_1608Metric",
        [
            Pin(num="1", name="1", func=pin_types.PASSIVE),
            Pin(num="2", name="2", func=pin_types.PASSIVE),
        ],
    )


def make_capacitor_template() -> Part:
    return make_template(
        "CAPACITOR",
        "C",
        "Capacitor_SMD:C_0603_1608Metric",
        [
            Pin(num="1", name="1", func=pin_types.PASSIVE),
            Pin(num="2", name="2", func=pin_types.PASSIVE),
        ],
    )


def make_probe_head_template() -> Part:
    return make_template(
        "ACTIVE_PROBE_HEAD",
        "J",
        "MycoMIDI:ActiveProbeHeadPads_1x02",
        [
            Pin(num="1", name="REC_PAD", func=pin_types.PASSIVE),
            Pin(num="2", name="REF_PAD", func=pin_types.PASSIVE),
        ],
    )


def make_power_flag_template() -> Part:
    return make_template(
        "POWER_FLAG",
        "TP",
        "TestPoint:TestPoint_Pad_D1.0mm",
        [Pin(num="1", name="PWR", func=pin_types.PWROUT)],
    )


def make_cable_connector_template() -> Part:
    return make_template(
        "JST_SH_1X04",
        "J",
        "Connector_JST:JST_SH_BM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal",
        [
            Pin(num="1", name="VPROBE_IN", func=pin_types.PASSIVE),
            Pin(num="2", name="GND", func=pin_types.PASSIVE),
            Pin(num="3", name="BUF_OUT", func=pin_types.PASSIVE),
            Pin(num="4", name="REF_PASS", func=pin_types.PASSIVE),
        ],
    )


def make_adc_input_template() -> Part:
    return make_template(
        "ADS131M08_CHANNEL",
        "U",
        "Package_QFP:TQFP-32_7x7mm_P0.8mm",
        [
            Pin(num="1", name="AINP", func=pin_types.INPUT),
            Pin(num="2", name="AINN", func=pin_types.INPUT),
        ],
    )


def make_lmp7701_template() -> Part:
    return make_template(
        "LMP7701",
        "U",
        "Package_TO_SOT_SMD:SOT-23-5",
        [
            Pin(num="1", name="OUT", func=pin_types.OUTPUT),
            Pin(num="2", name="V-", func=pin_types.PWRIN),
            Pin(num="3", name="IN+", func=pin_types.INPUT),
            Pin(num="4", name="IN-", func=pin_types.INPUT),
            Pin(num="5", name="V+", func=pin_types.PWRIN),
        ],
    )


def main() -> None:
    resistor = make_resistor_template()
    capacitor = make_capacitor_template()
    power_flag = make_power_flag_template()
    probe_head = make_probe_head_template()(tag="probe_head")
    cable = make_cable_connector_template()(tag="cable")
    adc = make_adc_input_template()(tag="adc_channel")
    op_amp = make_lmp7701_template()(tag="buffer")
    vprobe_flag = power_flag(tag="vprobe_source")
    vprobe_local_flag = power_flag(tag="vprobe_local_source")
    gnd_flag = power_flag(tag="gnd_source")

    r_input = resistor(value="100k", tag="input_series")
    r_bias = resistor(value="47M", tag="input_bias_return")
    r_vbias_top = resistor(value="1M", tag="vbias_top")
    r_vbias_bottom = resistor(value="1M", tag="vbias_bottom")
    r_supply = resistor(value="22", tag="supply_filter")
    r_output = resistor(value="51", tag="output_isolation")

    c_vbias = capacitor(value="1uF", tag="vbias_bypass")
    c_decouple = capacitor(value="100nF", tag="local_decouple")
    c_bulk = capacitor(value="4.7uF", tag="local_bulk")

    vprobe_in = Net("VPROBE_IN")
    vprobe_local = Net("VPROBE_LOCAL")
    gnd = Net("GND")
    vbias = Net("VBIAS")
    rec = Net("REC_PAD")
    ref_pass = Net("REF_PASS")
    buf_in = Net("BUF_IN")
    out_raw = Net("BUF_OUT_RAW")
    buf_out = Net("BUF_OUT")

    vprobe_in += cable["VPROBE_IN"], vprobe_flag["PWR"]
    gnd += cable["GND"], op_amp["V-"], gnd_flag["PWR"]
    ref_pass += cable["REF_PASS"], probe_head["REF_PAD"], adc["AINN"]

    rec += probe_head["REC_PAD"], r_input["1"]
    buf_in += r_input["2"], op_amp["IN+"], r_bias["1"]
    vbias += r_bias["2"], r_vbias_top["2"], r_vbias_bottom["1"], c_vbias["1"]

    vprobe_in += r_supply["1"]
    vprobe_local += r_supply["2"], op_amp["V+"], c_decouple["1"], c_bulk["1"], r_vbias_top["1"], vprobe_local_flag["PWR"]
    gnd += c_decouple["2"], c_bulk["2"], r_vbias_bottom["2"], c_vbias["2"]

    out_raw += op_amp["OUT"], op_amp["IN-"], r_output["1"]
    buf_out += r_output["2"], cable["BUF_OUT"], adc["AINP"]

    ERC()
    generate_netlist(file_=str(OUTPUT_PATH), do_backup=False)
    print(f"Wrote {OUTPUT_PATH}")
    print("Active probe flex concept:")
    print("- recording electrode is buffered locally by an LMP7701 unity follower")
    print("- reference electrode passes straight through for ADS131M08 AINxN")
    print("- buffered output returns on BUF_OUT for ADS131M08 AINxP")


if __name__ == "__main__":
    main()
