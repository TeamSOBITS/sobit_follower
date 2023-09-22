実験前の手順
・被験者に指定の位置に立ってもらう && 動きの確認
・before_change
  →bagの名前を変更する
・カメラのセット

実験中の手順
・e1, e2, e3を実行, camera start

#
実験後の手順
・check_rosbag_name
  -rosbag名を確認
・after_change
  -rosbag名を変更する
・plot
・plot後の結果を確認

（実験後part2）
・rviz_followme
・cd catkin_ws/src && rosbag play "bag名"

命名規則の例
・2人の実験1回目（対象者-対象外1）
suzuki_mukogawa_1
・2人の実験2回目（対象者-対象外2）
suzuki_mukogawa_2
・3人の実験（対象者-対象外1-対象外2）
suzuki-mukogawa-kaneko

メモ(つけた名前を書いていく)
-- _1
-- 
--
--

