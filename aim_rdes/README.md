# aim_rdes

Realtime Deviation Encoding Scheme (RDES3) telemetry delta-compression library for microcontrollers and Python FFI tools. Saves **>50% flash space** with zero dynamic memory allocation.

---

## C/C++ Firmware Usage ([`src/rdes.h`](src/rdes.h))

```cpp
#include <rdes.h>

uint32_t current_row[4] = { millis(), p_pa, a_z, state };
uint32_t last_row[4]    = { 0 };
uint8_t  out_buf[20];

// Encodes row delta into out_buf (returns written byte count)
size_t len = rdes_encode_row_inline(out_buf, current_row, last_row, 4, false);
```

---

## Python Decompression ([`python/rdes_ctypes.py`](python/rdes_ctypes.py))

```python
from rdes_ctypes import RDESDecompressorCTypes

decompressor = RDESDecompressorCTypes(num_cols=4)
data_matrix  = decompressor.decompress(raw_bytes) # Returns List[List[int]]
```

---

## Shared Library Compile Command

Run from `av-libraries/aim_rdes`:

```bash
# Windows
cd av-libraries/aim_rdes
gcc -shared -O3 -I src -o python/librdes.dll src/rdes.c

# Linux / macOS
cd av-libraries/aim_rdes
gcc -shared -fPIC -O3 -I src -o python/librdes.so src/rdes.c
```
