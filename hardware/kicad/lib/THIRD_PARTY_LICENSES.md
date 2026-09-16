# Third-party code attributions for `hardware/kicad/lib`

MycoMIDI includes small, standalone ports adapted from MIT-licensed KiCad MCP
tooling. The files below retain their original upstream attribution in source
headers.

## ProductOfAmerica / mcp-server-kicad

- Repository: https://github.com/ProductOfAmerica/mcp-server-kicad
- License: MIT
- Ported files in this repo:
  - `hardware/kicad/lib/kicad_sexpr_cst.py`
    - adapted from `mcp_server_kicad/_cst.py`
  - `hardware/kicad/lib/atomic_write.py`
    - adapted from `mcp_server_kicad/_shared.py` (`_atomic_write`)

### MIT License

```text
MIT License

Copyright (c) 2026 ProductOfAmerica

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Osman Aslan / kicad-mcp-pro

- Repository: https://github.com/oaslananka/kicad-mcp-pro
- License: MIT
- Ported files in this repo:
  - `hardware/kicad/lib/drc_classify.py`
    - adapted from `src/kicad_mcp/validation/drc_runner.py`
    - classification ideas also informed by `src/kicad_mcp/validation/drc_report.py`
  - `hardware/kicad/lib/pin_header_footprint_gen.py`
    - adapted from `src/kicad_mcp/utils/footprint_gen.py` (`_pin_header`)

### MIT License

```text
MIT License

Copyright (c) 2026 Osman Aslan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
