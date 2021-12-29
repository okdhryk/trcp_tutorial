# トヨタライセンス版シミュレータを使う

トヨタライセンス版のHSRシミュレータでWRSの環境

hsr.ioのIDが無いとだめです
# インストール
1. Ubuntu 20.04をインストールする
2. ROS Noeticをインストールする
3. HSRポータルサイト（HSR.io）のHSRBユーザマニュアルに従って，開発PCのセットアップを行う


# シミュレータの起動
```
$ roslaunch hsrb_gazebo_launch hsrb_megaweb2015_world.launch
```
![image](https://user-images.githubusercontent.com/498658/147627793-a210b501-8b80-46e9-ba21-444717d090cc.png)

# WRS環境の利用
```
$ cd ~/
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ catkin_init_workspace
$ git clone https://github.com/hsr-project/hsrb_wrs_gazebo_launch.git
$ git clone https://git.hsr.io/wrs2020/tmc_wrs_gazebo_world.git
$ git clone https://github.com/hsr-project/tmc_gazebo_task_evaluators.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
$ rosrun tmc_gazebo_task_evaluators setup_score_widget
$ roslaunch hsrb_wrs_gazebo_launch wrs_practice0_tmc.launch
```
