import struct
import numpy as np

class FrameParser:
    """
    Parses radar data frames.
    Handles buffer management and extraction of point cloud data.
    """
    def __init__(self):
        self.buffer = bytearray()
        self.frame_count = 0
        self.MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'

    def parse(self, data):
        """
        Parses incoming data and returns a list of frames.
        """
        self.buffer.extend(data)
        frames = []
        
        while self.MAGIC_WORD in self.buffer:
            start_idx = self.buffer.find(self.MAGIC_WORD)
            if start_idx > 0:
                self.buffer = self.buffer[start_idx:]
            
            if len(self.buffer) < 40:
                break
                
            try:
                packet_len = struct.unpack('<I', self.buffer[12:16])[0]
            except Exception as e:
                print(f"Header parse error: {e}")
                break
            
            if len(self.buffer) < packet_len:
                break
            
            frame_data = self.buffer[:packet_len]
            self.buffer = self.buffer[packet_len:]
            
            parsed_frame = self._parse_frame(frame_data)
            if parsed_frame:
                frames.append(parsed_frame)
                self.frame_count += 1
                
        return frames

    def _parse_frame(self, frame_data):
        """
        Internal method to parse a single frame.
        Extracts frame_id and point cloud coordinates using TLV structure.
        """
        try:
            # Header parsing
            # Magic Word (8), Version (4), Total Packet Len (4), Platform (4), 
            # Frame Number (4), Time CPU Cycles (4), Num Detected Obj (4), Num TLVs (4), Subframe Number (4)
            # Total Header Len is typically 36 or 40 depending on version
            version = struct.unpack('<I', frame_data[8:12])[0]
            header_len = 40 if version > 0x01000005 else 36
            
            frame_id = struct.unpack('<I', frame_data[20:24])[0]
            num_tlvs = struct.unpack('<I', frame_data[32:36])[0]
            
            points = []
            idx = header_len
            
            for _ in range(num_tlvs):
                if idx + 8 > len(frame_data):
                    break
                    
                tlv_type, tlv_len = struct.unpack('<II', frame_data[idx:idx+8])
                
                if tlv_type == 1: # Detected Points
                    num_points = (tlv_len - 8) // 16
                    data_start = idx + 8
                    for i in range(num_points):
                        off = data_start + i * 16
                        if off + 16 > len(frame_data):
                            break
                        x, y, z, v = struct.unpack('<ffff', frame_data[off:off+16])
                        points.append({
                            'x': x,
                            'y': y,
                            'z': z,
                            'v': v,
                            'range': np.sqrt(x*x + y*y + z*z)
                        })
                
                idx += tlv_len
                
            return {
                'frame_id': frame_id,
                'num_points': len(points),
                'points': points
            }
        except Exception as e:
            print(f"Frame parsing error: {e}")
            return None
