# -- coding: utf-8 --
import sys
import os

# コマンドライン引数 引数1：入力ファイル(path) 引数2：出力ファイル名(path)
args = sys.argv
input_file=os.path.normpath(os.path.join(os.getcwd(),args[1]))
output_file=os.path.normpath(os.path.join(os.getcwd(),args[2]))
out_p2o = os.path.normpath(os.path.join(os.getcwd(),args[3]))
initial_pose=os.path.normpath(os.path.join(os.getcwd(),args[4]))

# ファイルの読み込み先頭から1行ずつ読み込む
with open(input_file, "r") as f:
    lines = f.readlines()

# 原点を配列に導入する。
ox_y_z = []
ox, oy, oz, o1, o2, o3, o4, o5 = map(float, lines[11].split())
ox_y_z.append([ox, oy, oz])

# 計算に用いる配列を記録する。
x_y_z = []
for i in range(12, len(lines)):
    x,y,z,x1,x2,x3,x4,x5 = map(float, lines[i].split())
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

# 初期位置の取得
# output.p2o_out.txt
with open(out_p2o, "r") as f:
    p2o = f.readlines()

# p2o ファイルの並進移動量を格納する。
p2ox_y_z = []
p2ox, p2oy, p2oz, p2oq1, p2oq2, p2oq3, p2oq4 = map(float, p2o[1].split())
p2ox_y_z.append([p2ox, p2oy, p2oz])

initx_y_z = []
initx = p2ox - ox
inity = p2oy - oy
initz = p2oz - oz
initx_y_z.append([initx,inity,initz,p2oq1,p2oq2,p2oq3,p2oq4])

with open(initial_pose, "w") as f:
    for row in initx_y_z:
        f.write(",".join(map(str, row)) + "\n")


# 出力確認 list 10個出力
# for i in range(0, 5):
#     print(x_y_z[i-1])
#     print(ox_y_z[0])
#     print(cx_y_z[i])
# print(len(cx_y_z))