ロボコンサークルで作ったプログラムです。  
"smach"というライブラリを使ってロボットの状態遷移を定義します。  
ROS(Robot Operating System)というソフトウェアプラットフォーム上で動くコードです。  
コメントを日本語で書くとエラーが出るので、英語で書いています。  
  
publisher_node.pyを実行すると数値の入力待ちになります。  
0もしくは1を入力します。  
  
smach_simple4.pyで状態遷移を定義しています。  
最初にwait --> initialize_modeと遷移し、先ほどのpublisher_nodeの入力値に応じて遷移先を決定します。  
  
入力値が０の場合：  
　　プログラムを終了。  
入力値が１の場合：  
　　initialize --> searchと遷移し、その後
    search --> harvest --> storage --> prepare --> search ...  
　　のループを３回繰り返したのち、プログラムを終了。  

https://github.com/rrrr6725/action-selection/assets/84482770/030f302d-0c94-446a-94e6-79a752911dae

    
  参考文献：  
  https://qiita.com/srs/items/3f5acc64a2faac48b63d   
  
（参考）ROSについて  
 https://wiki.ros.org/ja  
   
（参考）smachについて  
 https://wiki.ros.org/ja/smach






