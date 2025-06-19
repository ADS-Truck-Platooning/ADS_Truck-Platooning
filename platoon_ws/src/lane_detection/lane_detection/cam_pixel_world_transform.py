import math
import numpy as np

def convert_pixel_to_world(u, v, cx=320, cy=240, fov_x=90.0, x=2.3, y=0.0, z=3.5, pitch=-45.0):
    # ───── 기본 파라미터 ─────
    # cx, cy   = 320, 240           # principal point
    # fov_x    = 90.0               # [deg]
    fx = fy = cx / math.tan(math.radians(fov_x / 2))

    cam_pos  = np.array([x, y, z])   # [m]  (0,0,3.5)

    pitch = math.radians(pitch)            # ↓ 45°
    Rx = np.array([[1, 0, 0],
                [0,  math.cos(pitch), -math.sin(pitch)],
                [0,  math.sin(pitch),  math.cos(pitch)]])

    align = np.array([[ 0,  0,  1],
                    [-1,  0,  0],
                    [ 0, -1,  0]])

    R_c2w = align @ Rx                     # 카메라 → 월드

    def pixel_to_world(u, v):
        # 1) 픽셀 → 정규화 시선벡터(카메라)
        d_c = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        d_c /= np.linalg.norm(d_c)

        # 2) 월드로 회전
        d_w = R_c2w @ d_c

        # 3) 지면(z=0)과 교차
        t = -cam_pos[2] / d_w[2]
        return cam_pos + t * d_w           # (X, Y, Z)

    coordinate = (tuple(pixel_to_world(u, v)[0:2]))
    return coordinate

def convert_world_to_pixel(X, Y, Z=0.0, cx=320, cy=240, fov_x=90.0, x=2.3, y=0.0, z=3.5, pitch=-45.0):
    """
    월드 좌표 (X,Y,[Z=0]) → 픽셀 좌표 (u,v) 변환
    - 월드 평면 z=0(지면)을 가정
    - 카메라 파라미터는 convert_pixel_to_world와 동일하게 둠
    """
    # ───── 기본 파라미터 ─────
    fx = fy = cx / math.tan(math.radians(fov_x / 2))  # focal length[pixel]

    cam_pos = np.array([x, y, z])                     # 카메라 위치 [m]

    pitch_rad = math.radians(pitch)
    Rx = np.array([[1, 0, 0],
                   [0,  math.cos(pitch_rad), -math.sin(pitch_rad)],
                   [0,  math.sin(pitch_rad),  math.cos(pitch_rad)]])

    align = np.array([[ 0,  0,  1],                  # OpenCV ↔ ROS 축 정렬
                      [-1,  0,  0],
                      [ 0, -1,  0]])

    R_c2w = align @ Rx                                # 카메라 → 월드
    R_w2c = R_c2w.T                                   # 월드 → 카메라 (정방향 행렬의 전치)

    # ───── 월드 → 카메라 ─────
    Pw = np.array([X, Y, Z])                        # 지면 상의 점 (Z=0)
    Pc = R_w2c @ (Pw - cam_pos)                       # 카메라 좌표계 점

    # 투영면(이미지) 뒤에 있으면 반환 불가
    if Pc[2] <= 0:
        raise ValueError("점이 이미지 평면 뒤에 있습니다. (Pc.z <= 0)")

    # ───── 카메라 → 픽셀 ─────
    u = fx * (Pc[0] / Pc[2]) + cx
    v = fy * (Pc[1] / Pc[2]) + cy

    return (u, v)
