# -- coding: utf-8 --
import sys
import os
import numpy as np

# コマンドライン引数 引数1：入力ファイル(path) 引数2：出力ファイル名(path) 引数3：p2o出力ファイル(path) 引数4：初期ポーズ出力ファイル(path) 引数5：初期緯度経度高度出力ファイル(path)
args = sys.argv
input_file = os.path.normpath(os.path.join(os.getcwd(), args[1]))
output_file = os.path.normpath(os.path.join(os.getcwd(), args[2]))
out_p2o = os.path.normpath(os.path.join(os.getcwd(), args[3]))
initial_pose = os.path.normpath(os.path.join(os.getcwd(), args[4]))
initial_lat_lon_alt = os.path.normpath(os.path.join(os.getcwd(), args[5]))

try:
    # ファイルの読み込み、先頭から11行目までを文字列として保持
    with open(input_file, "r") as f:
        header_lines = [next(f) for _ in range(11)]
        # 原点の読み込みと NumPy 配列への変換
        origin_line = next(f).split()
        if origin_line:
            origin = np.array(list(map(float, origin_line)))
        else:
            print("エラー：原点の行が空です。")
            sys.exit(1)
        # 残りのデータを NumPy 配列として一括読み込み
        data = np.loadtxt(f)
except FileNotFoundError as err:
    print(err)
    print('入力ファイルが見つかりません。')
    sys.exit(1)
except ValueError as err:
    print(err)
    print('入力ファイルの数値形式が不正です。')
    sys.exit(1)
except StopIteration:
    print("エラー：入力ファイルの形式が不正です。原点の行が見つかりません。")
    sys.exit(1)

print("点群の平行移動を開始します。")

# 座標の計算 (原点を含む)
calculated_data = np.vstack([[0.0, 0.0, 0.0], data - origin])

print("点群の平行移動を終了しました。")

# 結果をファイル出力
with open(output_file, "w") as f:
    # 文字列部を書き込み
    f.writelines(header_lines)
    # データ部を NumPy 配列から文字列に変換して一括書き込み
    np.savetxt(f, calculated_data, fmt='%f')

try:
    # p2o ファイルの読み込み
    with open(out_p2o, "r") as f:
        p2o_lines = f.readlines()
        if len(p2o_lines) < 2:
            print("エラー：p2o ファイルの行数が不足しています。")
            sys.exit(1)
        p2o_data = np.array([list(map(float, line.split())) for line in p2o_lines])
        if p2o_data.shape[0] < 2 or p2o_data.shape[1] < 7:
            print("エラー：p2o ファイルのデータ形式が不正です。")
            sys.exit(1)
        p2o_origin = p2o_data[1, :3]  # p2o の原点 (2行目の最初の3要素)
        initial_lat_lon_alt_data = p2o_data[0, 7:] # p2o の初期緯度経度高度 (1行目の後半3要素)
        initial_quat = p2o_data[1, 3:7] # p2o の初期クォータニオン (2行目の4-7要素)

except FileNotFoundError as err:
    print(err)
    print('p2o ファイルが見つかりません。')
    sys.exit(1)
except ValueError as err:
    print(err)
    print('p2o ファイルの数値形式が不正です。')
    sys.exit(1)
except IndexError as err:
    print(err)
    print('p2o ファイルの形式が想定外です。')
    sys.exit(1)

# 初期位置の計算とファイル出力
initial_translation = p2o_origin - origin
initial_pose_data = np.concatenate([initial_translation, initial_quat])
with open(initial_pose, "w") as f:
    f.write(",".join(map(str, initial_pose_data)) + "\n")

with open(initial_lat_lon_alt, "w") as f:
    f.write(",".join(map(str, initial_lat_lon_alt_data)) + "\n")

# デバッグ用出力 (最初の数点と形状の確認)
if 'DEBUG' in os.environ:
    if len(data) > 5:
        print("Original Data (first 5):")
        print(data[:5])
        print("Origin:")
        print(origin)
        print("Calculated Data (first 5):")
        print(calculated_data[:5])
    print(f"Shape of Original Data: {data.shape}")
    print(f"Shape of Calculated Data: {calculated_data.shape}")