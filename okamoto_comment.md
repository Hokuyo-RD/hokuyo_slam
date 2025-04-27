# p2o_from_rosbag.py
おそらく問題ない。  
EDGE_LLA(緯度経度) と EDGE_LIN3D(平面直角座標系) の対応は取れていそう。

# rearrange_pointcloud.cpp
問題ない。

# run_p2o.cpp
問題あり。
- [112~114行目]  
    result[] は（おそらく）LIOの軌跡をグラフベースで修正したものなので、EDGE_LLAの各要素とは対応しない。
- 対応策  
    114行目のlat,lonには、result[i].x,result[i].y をPROJライブラリで緯度経度に変換したものを使用する。

# pcd_to_Rcord.py
問題はなさそう。以下はメモ
- 内容整理
    - 各変数
        - p2ox_y_z:  
            p2oファイル1行目の平面直角座標 
        - ox_y_z:  
            p2oファイル10行目の平面直角座標 
        - oxyz:  
            pcdファイルの一点目の平面直角座標 
        - xyz[]:  
            pcdファイルの各要素
        - lat,lon,alt:  
            p2oファイル10行目の緯度経度高度
    - 平行移動処理 [56行目]
        - cx_y_z[i+1] = [x - y for x, y in zip(x_y_z[i], ox_y_z[0])]  
        pcdファイルをp2oファイルの10行目の座標で平行移動している
    - 原点のテキスト出力処理 [68行目~]
        - p2ox_y_z - ox_y_z
            - p2oファイル一行目と10行目の平面直角座標の差  
                別に　0,0,0　,0,0,0,1　でいい気がする
        - init_lat_lon_alt.append([lat,lon,alt])
            - これは問題なさそう 
