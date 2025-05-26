import pyqrcode
import cv2

# QRコード作成(文字列決定,エラー訂正レベル(H>Q>M>L),データ量によるサイズ指定(40~1),エンコード方法)
code = pyqrcode.create('turn_right', error='L', version=1, mode='binary')
# QRコード保存(名称,サイズ,コード色,背景色)
code.png('qrcode_R1.png', scale=5, module_color=[0, 0, 0, 128], background=[255, 255, 255])

# QRコード作成
code = pyqrcode.create('turn_left', error='L', version=1, mode='binary')
code.png('qrcode_L1.png', scale=5, module_color=[0, 0, 0, 128], background=[255, 255, 255])

# QRコード作成
code = pyqrcode.create('stop_end', error='L', version=1, mode='binary')
code.png('qrcode_E1.png', scale=5, module_color=[0, 0, 0, 128], background=[255, 255, 255])