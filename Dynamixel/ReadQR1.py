import numpy as np
import cv2

# QRコード一辺の長さ
marker_size = 0.03 # [m]
# カメラの内部パラメータと歪み係数(chess boardから取得)
camera_matrix = np.array([[786.38858756 ,  0.         ,351.02240753],
                          [  0.         ,788.85699087 ,260.48178893],
                          [  0.         ,  0.         ,  1.        ]])
distortion_coeff = np.array([-0.03169828 ,-0.16365523  ,0.0051104  ,-0.00278013  ,0.50978427])

## rvec -> rotation vector, tvec -> translation vector
def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
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
    font = cv2.FONT_HERSHEY_SIMPLEX
    # -----------------------------------------------------------
    # 画像キャプチャ
    # -----------------------------------------------------------
    # VideoCaptureインスタンス生成
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) #for Win
    # cap = cv2.VideoCapture(0) #for Ubuntu/Raspberry Pi
    
    # QRCodeDetectorインスタンス生成
    qrd = cv2.QRCodeDetector()

    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            # QRコードデコード
            retval, decoded_info, points, straight_qrcode = qrd.detectAndDecodeMulti(frame)

            if retval and points is not None:
                points = points.astype(np.int32)
                for dec_inf, point in zip(decoded_info, points):
                    if dec_inf == '':
                        continue

                    # QRコード各頂点座標を取得
                    x, y = point[0][0], point[0][1]

                    #頂点座標から距離,角度計算
                    rvec, tvec,_ = my_estimatePoseSingleMarkers(points, marker_size, camera_matrix, distortion_coeff)
                    # < rodoriguesからeuluerへの変換 >
                    # 不要なaxisを除去
                    tvec = np.squeeze(tvec)
                    rvec = np.squeeze(rvec)
                    # 回転ベクトルからrodoriguesへ変換
                    rvec_matrix = cv2.Rodrigues(rvec)
                    rvec_matrix = rvec_matrix[0] # rodoriguesから抜き出し
                    # 並進ベクトルの転置
                    transpose_tvec = tvec[np.newaxis, :].T
                    # 合成
                    proj_matrix = np.hstack((rvec_matrix, transpose_tvec))
                    # オイラー角への変換
                    euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6] # [deg]
                    
                    # コンソールに情報を表示
                    print("="*50)
                    print(f"QRコードデータ: {dec_inf}")
                    print(f"位置情報:")
                    print(f"  x: {tvec[0]:.4f} [m]")
                    print(f"  y: {tvec[1]:.4f} [m]")
                    print(f"  z: {tvec[2]:.4f} [m]")
                    print(f"姿勢情報:")
                    print(f"  roll : {euler_angle[0][0]:.2f} [deg]")
                    print(f"  pitch: {euler_angle[1][0]:.2f} [deg]")
                    print(f"  yaw  : {euler_angle[2][0]:.2f} [deg]")

                    # 条件分岐（情報表示のみ）
                    if dec_inf == "turn_right":
                        print("QRコードの種類: turn_right")
                        print(f"頂点座標: {point}")
                        print("アクション: 右回転指示")
                        frame = cv2.putText(frame, "Turn Right", (x, y - 10), font, 0.7, (255, 0, 0), 2, cv2.LINE_AA)

                    elif dec_inf == "turn_left":
                        print("QRコードの種類: turn_left")
                        print(f"頂点座標: {point}")
                        print("アクション: 左回転指示")
                        frame = cv2.putText(frame, "Turn Left", (x, y - 10), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

                    elif dec_inf == "stop_end":
                        print("QRコードの種類: stop_end")
                        print(f"頂点座標: {point}")
                        print("アクション: 停止指示")
                        frame = cv2.putText(frame, "Stop End", (x, y - 10), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

                    else:
                        print("QRコードの種類: その他")
                        print(f"頂点座標: {point}")
                        print("アクション: 不明なQRコード")
                        frame = cv2.putText(frame, "Unknown QR", (x, y - 10), font, 0.7, (255, 255, 0), 2, cv2.LINE_AA)

                    print("="*50)
                    
                    # QRコードに枠線を描画
                    frame = cv2.polylines(frame, [point], True, (0, 255, 0), 2, cv2.LINE_AA)
                    
            # 画像表示
            cv2.imshow('QR Code Detection', frame)
            
        # Qキーを押して終了
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    # キャプチャリソースリリース
    cap.release()
    cv2.destroyAllWindows()

#実行
if __name__ == '__main__':
    main()
