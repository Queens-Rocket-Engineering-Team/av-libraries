import ctypes
import os
import sys

class RDESDecompressorCTypes:
    """
    Python FFI wrapper for the RDES shared C library (librdes.dll / librdes.so).
    Decompresses binary flight logs into 2D numerical telemetry matrices.
    """
    def __init__(self, num_cols=None, numCols=None):
        self.num_cols = num_cols if num_cols is not None else numCols
        if not self.num_cols:
            raise ValueError("num_cols must be specified.")
            
        self.lib = self._load_lib()
        
        self.lib.rdes_decompress_log.argtypes = [
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_size_t,
            ctypes.c_size_t
        ]
        self.lib.rdes_decompress_log.restype = ctypes.c_size_t

    def _load_lib(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        ext = ".dll" if sys.platform == "win32" else ".so"
        lib_path = os.path.join(dir_path, f"librdes{ext}")
        
        try:
            return ctypes.CDLL(lib_path)
        except OSError as e:
            raise RuntimeError(
                f"Failed to load RDES shared library at '{lib_path}'. "
                f"Ensure librdes{ext} is compiled for your platform (e.g. using 'gcc -shared -O3 -o librdes{ext} rdes.c'). Error: {e}"
            )

    def decompress(self, raw_data):
        if not raw_data:
            return []
        
        max_rows = len(raw_data)
        in_arr = (ctypes.c_uint8 * len(raw_data)).from_buffer_copy(raw_data)
        out_arr = (ctypes.c_uint32 * (max_rows * self.num_cols))()
        
        row_count = self.lib.rdes_decompress_log(
            in_arr, len(raw_data), out_arr, max_rows, self.num_cols
        )
        
        result = []
        for i in range(row_count):
            start = i * self.num_cols
            result.append(list(out_arr[start:start + self.num_cols]))
        return result
