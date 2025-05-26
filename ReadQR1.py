import numpy as np
import cv2
import time

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
    
    # 前回検出したQRコードを記録（重複表示防止）
    last_detected_qr = ""
    last_detection_time = 0
    detection_cooldown = 1.0  # 1秒間のクールダウン
    
    print("QRコード検出システムを開始しています...")
    print("'q'キーを押すと終了します")
    print("="*60)
    
    # -----------------------------------------------------------
    # 画像キャプチャ
    # -----------------------------------------------------------
    # VideoCaptureインスタンス生成
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) #for Win
    # cap = cv2.VideoCapture(0) #for Ubuntu/Raspberry Pi
    
    if not cap.isOpened():
        print("エラー: カメラを開けませんでした")
        return
    
    # QRCodeDetectorインスタンス生成
    qrd = cv2.QRCodeDetector()
    
    frame_count = 0
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("フレームの取得に失敗しました")
                break
            
            frame_count += 1
            current_time = time.time()
            
            # QRコードデコード
            retval, decoded_info, points, straight_qrcode = qrd.detectAndDecodeMulti(frame)

            # 待機状態の表示（10フレームごと）
            if frame_count % 30 == 0 and not retval:
                print("QRコードを待機中...", end='\r')
            
            if retval and points is not None:
                points = points.astype(np.int32)
                
                for dec_inf, point in zip(decoded_info, points):
                    if dec_inf == '':
                        continue
                    
                    # 重複検出防止（同じQRコードを短時間で連続検出した場合）
                    if (dec_inf == last_detected_qr and 
                        current_time - last_detection_time < detection_cooldown):
                        continue
                    
                    # 新しいQRコードまたは時間経過後の再検出
                    last_detected_qr = dec_inf
                    last_detection_time = current_time
                    
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
                    print("\n" + "="*60)
                    print(f"🎯 QRコード検出！ [{time.strftime('%H:%M:%S')}]")
                    print("="*60)
                    print(f"QRコードデータ: {dec_inf}")
                    print(f"位置情報:")
                    print(f"  x: {tvec[0]:.4f} [m]")
                    print(f"  y: {tvec[1]:.4f} [m]")
                    print(f"  z: {tvec[2]:.4f} [m]")
                    print(f"姿勢情報:")
                    print(f"  roll : {euler_angle[0][0]:.2f} [deg]")
                    print(f"  pitch: {euler_angle[1][0]:.2f} [deg]")
                    print(f"  yaw  : {euler_angle[2][0]:.2f} [deg]")
                    print(f"頂点座標: {point}")

                    # 条件分岐（情報表示のみ）
                    if dec_inf == "turn_right":
                        print("📍 QRコードの種類: turn_right")
                        print("🔄 アクション: 右回転指示")
                        frame = cv2.putText(frame, "Turn Right", (x, y - 10), font, 0.7, (255, 0, 0), 2, cv2.LINE_AA)

                    elif dec_inf == "turn_left":
                        print("📍 QRコードの種類: turn_left")
                        print("🔄 アクション: 左回転指示")
                        frame = cv2.putText(frame, "Turn Left", (x, y - 10), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

                    elif dec_inf == "stop_end":
                        print("📍 QRコードの種類: stop_end")
                        print("🛑 アクション: 停止指示")
                        frame = cv2.putText(frame, "Stop End", (x, y - 10), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

                    else:
                        print("📍 QRコードの種類: その他")
                        print("❓ アクション: 不明なQRコード")
                        frame = cv2.putText(frame, "Unknown QR", (x, y - 10), font, 0.7, (255, 255, 0), 2, cv2.LINE_AA)

                    print("="*60)
                    print("QRコードを待機中...")
                    
                    # QRコードに枠線を描画
                    frame = cv2.polylines(frame, [point], True, (0, 255, 0), 2, cv2.LINE_AA)
                    
            # 画像表示
            cv2.imshow('QR Code Detection', frame)
            
            # Qキーを押して終了
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\nプログラムを終了します...")
                break
                
    except KeyboardInterrupt:
        print("\nキーボード割り込みでプログラムを終了します...")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        # キャプチャリソースリリース
        cap.release()
        cv2.destroyAllWindows()
        print("リソースを解放しました")

#実行
if __name__ == '__main__':
    main()
