## analyze-shot-log
ベルト直動のログを解析するリポジトリです。

## 前準備
リポジトリのclone 
```bash 
git clone https://github.com/tmcit-ararobo-2026a/analyze-shot-log.git
```
submodule を反映
```bash
git submodule init
git submodule update
```

## 解析手順

```
☆[jsonからcsvに変換]　---> [csvからmodelを作成] --> [modelを用いて目標距離から速度を求める]
```

### step1 jsonからcsvに変換 (convert)
```bash
python3 launch-model.py convert trials.jsonl -o out --include-all --target 4:5.0,0.0
```

| パラメータ | 説明 |
| :--- | :---: |
|``trials.jsonl``	| 元データ(入力)|  
|``-o out`` | 出力先フォルダ。ここに out/shots.csv ができます |   
|``--include-all`` | 速度の範囲外データも、消さずに残します(範囲の選択については後ほど記述。初期範囲3.00 ~ 7.75) |
|``--target 4:5.0,0.0`` |  ID 4 の座標を x=5.0 m, y=0.0 m と教えます。誤差の計算に使います。|

もしこのコマンドを使ったら、``out``というディレクトリ直下に``shots.csv``ができるはずです。

### step2 csvからmodelを作成 (fit)
上記のcsvを用いて、データに式を当てはめて、式の係数を決めます。
```bash
python3 launch-model.py fit out/shots.csv --model-out out/model.json --use-all
```  
| パラメータ | 説明 |
| :--- | :---: |
|``out/shots.csv`` | 手順1で作ったCSV(入力) | 
|``--model-out out/model.json`` | 作ったモデルの保存先ファイル | 
|``--use-all`` | 範囲外のデータも学習に使う(step1で``include-all``をしていなかったらつけない)  |


### 



