import ctypes
import os
import sys

class RDESDecompressorCTypes:
    """
    Python wrapper for the RDES shared C library (librdes.dll / librdes.so).
    Uses C99 rdes.h directly as the single source of truth for RDES3 encoding.
    Full attribution to Kennan (Kenneract) for the original algorithm.
    """
    def __init__(self, num_cols=None, numCols=None):
        self.num_cols = num_cols if num_cols is not None else numCols
        if not self.num_cols:
            raise ValueError("num_cols must be specified.")
            
        self.lib = self._load_lib()
        
        self.lib.rdes_decode_row.argtypes = [
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_size_t
        ]
        self.lib.rdes_decode_row.restype = ctypes.c_size_t
        
        self.lib.rdes_encode_row.argtypes = [
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_size_t,
            ctypes.c_bool
        ]
        if hasattr(self.lib, 'rdes_decompress_log'):
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
            
    def encode_row(self, current_row, last_row, force_origin=False):
        out_buf = (ctypes.c_uint8 * (self.num_cols * 5))()
        cur_arr = (ctypes.c_uint32 * self.num_cols)(*current_row)
        last_arr = (ctypes.c_uint32 * self.num_cols)(*last_row)
        bytes_written = self.lib.rdes_encode_row(out_buf, cur_arr, last_arr, self.num_cols, force_origin)
        return bytes(out_buf[:bytes_written])

    def decode_row(self, in_buf, last_row):
        in_arr = (ctypes.c_uint8 * len(in_buf))(*in_buf)
        cur_arr = (ctypes.c_uint32 * self.num_cols)()
        last_arr = (ctypes.c_uint32 * self.num_cols)(*last_row)
        bytes_read = self.lib.rdes_decode_row(in_arr, cur_arr, last_arr, self.num_cols)
        return list(cur_arr), bytes_read

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
