import numpy as np
import cv2
from picamera2 import Picamera2
import numpy as np


# QRコード一辺の長さ [m]
marker_size = 0.03

# リアルタイムの画像表示の有り無し
Image_preview = 1  # 画像の表示をする場合は1,しない場合(light)は0

# カメラの内部パラメータ
camera_matrix = np.array([
    [1.99927263e+03, 0.00000000e+00, 3.30848333e+02],
    [0.00000000e+00, 1.99806663e+03, 2.57552152e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

# 歪み係数
distortion_coeff = np.array([
    -2.46051320e-01,
     3.25855770e+01,
     1.15522156e-02,
     6.92733918e-03,
    -7.35148202e+02
])


def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    marker_points = np.array([
        [-marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0]
    ], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    corners = corners.astype('float32')
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash

def main():
    picam2 = Picamera2()
    picam2.start()
    qrd = cv2.QRCodeDetector()

    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        retval, decoded_info, points, _ = qrd.detectAndDecodeMulti(frame)

        if retval and points is not None:
            points = points.astype(np.int32)
            for dec_inf, point in zip(decoded_info, points):
                if dec_inf == '':
                    continue

                if Image_preview == 1:
                    # QRコードの枠線を描画
                    frame = cv2.polylines(frame, [point], True, (0, 255, 0), 2, cv2.LINE_AA)

                # 姿勢推定
                rvec, tvec, _ = my_estimatePoseSingleMarkers(points, marker_size, camera_matrix, distortion_coeff)
                tvec = np.squeeze(tvec)
                rvec = np.squeeze(rvec)
                rvec_matrix = cv2.Rodrigues(rvec)[0]
                transpose_tvec = tvec[np.newaxis, :].T
                proj_matrix = np.hstack((rvec_matrix, transpose_tvec))
                euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6]

                # ここで処理（例：print、他の関数に渡すなど）
                print("QRデータ:", dec_inf)
                print("x:", tvec[0], "y:", tvec[1], "z:", tvec[2])
                print("roll:", euler_angle[0], "pitch:", euler_angle[1], "yaw:", euler_angle[2])
                print("-" * 40)

            if Image_preview == 1:
                # 画像を表示
                cv2.imshow('cv2', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            # qが入力されたら終了
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

# 変更テスト