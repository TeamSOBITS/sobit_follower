# sobit_follower2
sobit_mini，sobit_edu用のFollowMe(人追従走行)パッケージ  

## Before Use  
1. [sobit_mini](https://gitlab.com/TeamSOBITS/sobit_mini) or [sobit_education](https://gitlab.com/TeamSOBITS/sobit_education)をgit clone
2. [ssd_node](https://gitlab.com/TeamSOBITS/ssd_node)をgit clone

## How to Use  

### * Follow Me
実機でFollow Meを行うlaunch  
```bash
$  roslaunch sobit_follower2 follow_me.launch 
```


## * SensorData Publisher
デバック用のセンサデータをパブリッシュするlaunch  
```bash
$  roslaunch sobit_follower2 sensor_data_publisher.launch
```


## * SensorData Saver
デバック用のセンサデータを保存するlaunch  
```bash
$  roslaunch sobit_follower2 sensor_data_saver.launch
```
※ 既に6フレーム分のセンサデータをsobit_follower2/sensor_dataに保存済み


### * Local Path Planning
DWAによる経路生成をシミュレートできるlaunch  
```bash
$  roslaunch sobit_follower2 local_path_planning_simulator.launch 
```

<div align="center">
![tracker](doc/simulator.gif)  

・緑の円：障害物の位置  
・赤の円：障害物のコスト  
・青の丸：目的地の位置  
・緑の線：予測された経路の候補  
・赤の線：障害物にぶつかる経路  
・青の線：選択された経路  
</div>

### FollowMeの仕組み
RGB-Dセンサにより人を捉え、LRFにより人を位置を把握し，人の位置までDynamic Window Approach(DWA)による経路生成を行い走行する  

#### RGB-Dセンサによる人認識
Single Shot Multibox Detector(SSD)により、人認識を行う  

[SSDとは](https://blog.negativemind.com/2019/02/26/general-object-recognition-single-shot-multibox-detector/)  
[package : ssd_node](https://gitlab.com/TeamSOBITS/ssd_node)  

#### LRFによる人位置追跡
Particle Filterを用いて、人位置追跡を行う  

[Particle Filterとは1](http://pointclouds.org/documentation/tutorials/tracking.php)  
[Particle Filterとは2](https://algorithm.joho.info/image-processing/particle-filter-tracking/)  

#### DWAによる経路生成
[DWAとは](https://myenigma.hatenablog.com/entry/20140624/1403618922)  