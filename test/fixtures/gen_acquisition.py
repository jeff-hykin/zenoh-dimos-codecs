"""Generate tiny fixtures with the acquisition-stamped dimos-lcm schemas.

Run with the sim2 DimOS venv (Image hash 3b5fa3414d592aee,
PointCloud2 hash ddede614f08f054c). Legacy fixtures remain separate.
"""
from pathlib import Path
import struct
from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.sensor_msgs import Image, PointCloud2, PointField

out = Path(__file__).parent / 'dimos'
header = SampleHeader(clock_id='sim-proof', epoch=3, time_ns=0,
                      source_id='camera', sequence=2**60 + 7, frame_id='g1/camera')
for name, encoding, data, step in [
    ('image_rgb8', 'rgb8', bytes([255, 0, 0, 0, 255, 0]), 6),
    ('depth_32FC1', '32FC1', struct.pack('<ff', 1.5, 2.5), 8),
]:
    image = Image(data_length=len(data), header=header, height=1, width=2,
                  encoding=encoding, is_bigendian=0, step=step, data=data)
    assert image._get_packed_fingerprint().hex() == '3b5fa3414d592aee'
    (out / f'acquisition_{name}.bin').write_bytes(image.lcm_encode())
header.source_id, header.frame_id = 'mid360', 'world'
data = struct.pack('<ffffff', 1, 2, 3, -1, -2, -3)
cloud = PointCloud2(fields_length=3, data_length=len(data), header=header,
    height=1, width=2, fields=[PointField(name=n, offset=i*4, datatype=7, count=1)
    for i, n in enumerate(('x', 'y', 'z'))], is_bigendian=0, point_step=12,
    row_step=24, data=data, is_dense=1)
assert cloud._get_packed_fingerprint().hex() == 'ddede614f08f054c'
(out / 'acquisition_pointcloud_xyz.bin').write_bytes(cloud.lcm_encode())
