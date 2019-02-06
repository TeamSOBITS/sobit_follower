# sobit_follower

#人追従の試作版
必要なライブラリとして，Eigenとturtlebot_eduが必要(まだ，インストール方法はまだ書いていません)

#人追従のセットアップ手順

# 1.Eigenをインストール
まだ，簡易なインストール方法がわからないのですが，一応載せておきます
下のURLに入る
http://eigen.tuxfamily.org/index.php?title=Main_Page

右上の"Get it"内の"The latest stable release"のtar.gzをクリック→ダウンロード


# 2.turtlebot_eduのインストール

./install_turtlebot_edu.sh

cd ~/catkin_make

catkin_make

#人追従プログラムの実行コマンド

roscd sobit_follower

chmod 755 *

roslaunch turtlebot_edu minimal.launch

roslaunch sobit_follower sobit_follower_test_2.launch

#セットアップの進捗状況
interactiveと混ざっている状況なので．いらないファイルが多いです。(後に整理しておきます)

