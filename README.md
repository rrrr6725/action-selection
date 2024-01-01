ロボコンサークルで作ったプログラムです。
ロボットの行動選択を定義します。
ROS(Robot Operating System)というソフトウェアプラットフォーム上で動くコードです。

publisher_node.pyを実行すると数値の入力待ちになります。
0もしくは1を入力します。

smach_simple4.pyで状態遷移を定義しています。
最初にwait --> initialize_modeと遷移し、先ほどのpublisher_nodeの入力値に応じて遷移先を決定します。

入力値が０の場合：
　　プログラムを終了。
入力値が１の場合：
　　initialize --> search --> harvest --> storage --> prepare -->initialize ...
　　のループを３回繰り返したのち、プログラムを終了。
