# -- coding: utf-8 --
import sys
import os

# コマンドライン引数 引数1：入力ファイル(path) 引数2：出力ファイル名(path)
args = sys.argv
input_file=os.path.normpath(os.path.join(os.getcwd(),args[1]))
output_file=os.path.normpath(os.path.join(os.getcwd(),args[2]))

# ファイルの読み込み先頭から1行ずつ読み込む
with open(input_file, "r") as f:
    lines = f.readlines()

# 原点を配列に導入する。
ox_y_z = []
ox, oy, oz = map(float, lines[11].split())
ox_y_z.append([ox, oy, oz])

# 計算に用いる配列を記録する。
x_y_z = []
for i in range(12, len(lines)):
    x,y,z = map(float, lines[i].split())
    x_y_z.append([x,y,z])

# 座標の計算結果を格納する配列を確保 x_y_z + 1(原点)と同じ長さの配列
cx_y_z = [[] for _ in range(len(x_y_z)+1)]

# リスト同士の計算 内包表記
cx_y_z[0] = [x - y for x, y in zip(ox_y_z[0], ox_y_z[0])]
for i in range(0, len(x_y_z)):
    cx_y_z[i+1] = [x - y for x, y in zip(x_y_z[i], ox_y_z[0])]

# ファイル出力
with open(output_file, "w") as f:
    # 文字列部
    for i in range(0,11):
        f.writelines(lines[i])
    # データ部を読み込み
    for row in cx_y_z:
        f.write(" ".join(map(str, row)) + "\n")

# 出力確認 list 10個出力
# for i in range(0, 5):
#     print(x_y_z[i-1])
#     print(ox_y_z[0])
#     print(cx_y_z[i])
# print(len(cx_y_z))