# ros_actionlib_tutorials

- `src/fibonacci~` ExecuteCallback登録のサンプル
- `src/averaging~` GoalCallback登録のサンプル
- `scripts/fibonacci~` ExecuteCallback登録のサンプル（python）
- `scripts/gen_numbers.py` /random_numberトピックに乱数を投げ続けるノード

### Averagingサンプルの動かし方
```
roscore
rosrun actionlib_tutorials gen_numbers.py
rosrun actionlib_tutorials averaging_server
rosrun actionlib_tutorials averaging_client
```
