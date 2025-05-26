import numpy as np
import cv2
from picamera2 import Picamera2
import time

# QRコード一辺の長さ
marker_size = 0.03 # [m]

# カメラの内部パラメータと歪み係数
camera_matrix = np.array([[1.99927263e+03, 0.00000000e+00, 3.30848333e+02],
                          [0.00000000e+00, 1.99806663e+03, 2.57552152e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

distortion_coeff = np.array([-2.46051320e-01,
                             3.25855770e+01,
                             1.15522156e-02,
                             6.92733918e-03,
                            -7.35148202e+02])

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
    # 前回検出したQRコードを記録（重複表示防止）
    last_detected_qr = ""
    last_detection_time = 0
    detection_cooldown = 1.0  # 1秒間のクールダウン
    
    print("QRコード検出システムを開始しています（ヘッドレスモード）...")
    print("Ctrl+Cを押すと終了します")
    print("="*60)
    
    try:
        # PiCamera2インスタンス生成
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (640, 480)})
        picam2.configure(config)
        picam2.start()
        print("カメラを正常に開始しました")
        
    except Exception as e:
        print(f"エラー: カメラを開けませんでした - {e}")
        return
    
    # QRCodeDetectorインスタンス生成
    qrd = cv2.QRCodeDetector()
    
    frame_count = 0
    
    try:
        while True:
            # PiCamera2でフレームを取得
            frame = picam2.capture_array()
            
            # 色変換（PiCamera2はRGB、OpenCVはBGRを期待）
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            frame_count += 1
            current_time = time.time()
            
            # QRコードデコード
            retval, decoded_info, points, straight_qrcode = qrd.detectAndDecodeMulti(frame)

            # 待機状態の表示（30フレームごと）
            if frame_count % 30 == 0 and not retval:
                print("QRコードを待機中...", end='\r')
            
            if retval and points is not None:
                points = points.astype(np.int32)
                
                for dec_inf, point in zip(decoded_info, points):
                    if dec_inf == '':
                        continue
                    
                    # 重複検出防止
                    if (dec_inf == last_detected_qr and 
                        current_time - last_detection_time < detection_cooldown):
                        continue
                    
                    # 新しいQRコードまたは時間経過後の再検出
                    last_detected_qr = dec_inf
                    last_detection_time = current_time
                    
                    # 姿勢推定
                    rvec, tvec,_ = my_estimatePoseSingleMarkers(points, marker_size, camera_matrix, distortion_coeff)
                    
                    tvec = np.squeeze(tvec)
                    rvec = np.squeeze(rvec)
                    rvec_matrix = cv2.Rodrigues(rvec)
                    rvec_matrix = rvec_matrix[0]
                    transpose_tvec = tvec[np.newaxis, :].T
                    proj_matrix = np.hstack((rvec_matrix, transpose_tvec))
                    euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6]
                    
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

                    # 条件分岐
                    if dec_inf == "turn_right":
                        print("📍 QRコードの種類: turn_right")
                        print("🔄 アクション: 右回転指示")
                    elif dec_inf == "turn_left":
                        print("📍 QRコードの種類: turn_left")
                        print("🔄 アクション: 左回転指示")
                    elif dec_inf == "stop_end":
                        print("📍 QRコードの種類: stop_end")
                        print("🛑 アクション: 停止指示")
                    else:
                        print("📍 QRコードの種類: その他")
                        print("❓ アクション: 不明なQRコード")

                    print("="*60)
                    print("QRコードを待機中...")
                    
                    # 画像をファイルに保存（オプション）
                    # cv2.imwrite(f'qr_detected_{int(time.time())}.jpg', frame)
            
            # GUI表示は削除（ヘッドレスモード）
            # cv2.imshow('QR Code Detection', frame)  # この行をコメントアウト
            
            # 短い待機
            time.sleep(0.1)
                
    except KeyboardInterrupt:
        print("\nキーボード割り込みでプログラムを終了します...")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        try:
            picam2.stop()
            print("リソースを解放しました")
        except:
            pass

if __name__ == '__main__':
    main()
