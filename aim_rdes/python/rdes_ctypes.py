import ctypes
import os
import sys

class RDESDecompressorCTypes:
    """
    Python wrapper for the RDES shared library using ctypes.
    Full attribution to Kennan (Kenneract) for the original algorithm.
    """
    def __init__(self, num_cols):
        self.num_cols = num_cols
        self.lib = self._load_lib()
        
        if self.lib:
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
            self.lib.rdes_encode_row.restype = ctypes.c_size_t
    
    def _load_lib(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        ext = ".dll" if sys.platform == "win32" else ".so"
        lib_path = os.path.join(dir_path, f"librdes{ext}")
        
        try:
            return ctypes.CDLL(lib_path)
        except OSError:
            print(f"Warning: Could not load {lib_path}, falling back to pure Python implementation.")
            return None
            
    def encode_row(self, current_row, last_row, force_origin=False):
        if self.lib:
            out_buf = (ctypes.c_uint8 * (self.num_cols * 5))()
            cur_arr = (ctypes.c_uint32 * self.num_cols)(*current_row)
            last_arr = (ctypes.c_uint32 * self.num_cols)(*last_row)
            bytes_written = self.lib.rdes_encode_row(out_buf, cur_arr, last_arr, self.num_cols, force_origin)
            return bytes(out_buf[:bytes_written])
        else:
            return self._encode_row_pure_python(current_row, last_row, force_origin)

    def decode_row(self, in_buf, last_row):
        if self.lib:
            in_arr = (ctypes.c_uint8 * len(in_buf))(*in_buf)
            cur_arr = (ctypes.c_uint32 * self.num_cols)()
            last_arr = (ctypes.c_uint32 * self.num_cols)(*last_row)
            bytes_read = self.lib.rdes_decode_row(in_arr, cur_arr, last_arr, self.num_cols)
            return list(cur_arr), bytes_read
        else:
            return self._decode_row_pure_python(in_buf, last_row)

    def _encode_row_pure_python(self, current_row, last_row):
        # Fallback pure python implementation
        out_buf = bytearray()
        for i in range(self.num_cols):
            cur = current_row[i]
            last = last_row[i]
            sign_add = cur >= last
            offset = (cur - last) if sign_add else (last - cur)
            
            if offset <= 8191:
                b1 = 0x80 | (0x20 if sign_add else 0) | ((offset >> 8) & 0x1F)
                b2 = offset & 0xFF
                out_buf.extend([b1, b2])
            elif offset <= 1048575:
                b1 = 0xC0 | (0x10 if sign_add else 0) | ((offset >> 16) & 0x0F)
                b2 = (offset >> 8) & 0xFF
                b3 = offset & 0xFF
                out_buf.extend([b1, b2, b3])
            elif cur <= 0x7FFFFFFF:
                out_buf.extend([
                    (cur >> 24) & 0x7F,
                    (cur >> 16) & 0xFF,
                    (cur >> 8) & 0xFF,
                    cur & 0xFF
                ])
            else:
                out_buf.extend([
                    0xE0,
                    (cur >> 24) & 0xFF,
                    (cur >> 16) & 0xFF,
                    (cur >> 8) & 0xFF,
                    cur & 0xFF
                ])
        return bytes(out_buf)

    def _decode_row_pure_python(self, in_buf, last_row):
        ptr = 0
        current_row = []
        for i in range(self.num_cols):
            b1 = in_buf[ptr]
            if (b1 & 0x80) == 0:
                val = ((b1 & 0x7F) << 24) | (in_buf[ptr+1] << 16) | (in_buf[ptr+2] << 8) | in_buf[ptr+3]
                current_row.append(val)
                ptr += 4
            elif (b1 & 0xE0) == 0xE0:
                val = (in_buf[ptr+1] << 24) | (in_buf[ptr+2] << 16) | (in_buf[ptr+3] << 8) | in_buf[ptr+4]
                current_row.append(val)
                ptr += 5
            elif (b1 & 0xC0) == 0xC0:
                sign_add = (b1 & 0x10) != 0
                offset = ((b1 & 0x0F) << 16) | (in_buf[ptr+1] << 8) | in_buf[ptr+2]
                current_row.append(last_row[i] + offset if sign_add else last_row[i] - offset)
                ptr += 3
            elif (b1 & 0xC0) == 0x80:
                sign_add = (b1 & 0x20) != 0
                offset = ((b1 & 0x1F) << 8) | in_buf[ptr+1]
                current_row.append(last_row[i] + offset if sign_add else last_row[i] - offset)
                ptr += 2
        return current_row, ptr
