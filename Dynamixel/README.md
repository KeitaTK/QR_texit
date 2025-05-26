# README.md

## 諸注意

* [python対応バージョン](https://www.mathworks.com/content/dam/mathworks/mathworks-dot-com/support/sysreq/files/python-compatibility.pdf)
  - R2018a: 3.5, 3.6
  - R2022a: 3.8, 3.9
* `pip install pyserial`(pip3の場合あり)でPythonにpyserialをインストールする
* ドライバはOS標準でftdiのドライバが設定される
* デバイスドライバからドライバの設定でlatencyを1msに設定する
* ポート番号，Baudrateを適切に設定する
* IDは重複なく設定すること．Baudは1Mbps推奨
* テストプログラムでは，動いたときの破損に注意

## Matlabプログラム

* test_classPyDynamixel.m: Dynamixelのテストプログラム
* classPyDynamixel.m: クラスファイル

## その他

* Dynamixel.py
* dynamixel_sdk: Dynamixel SDKのファイルが入ったフォルダ
