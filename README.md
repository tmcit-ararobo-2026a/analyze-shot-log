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
|``-o out`` | 出力先フォルダ。ここに out/shots.csv ができる |   
|``--include-all`` | 速度の範囲外データも、消さずに残す(範囲の選択については後ほど記述。初期範囲3.00 ~ 7.75) |
|``--target 4:5.0,0.0`` |  ID 4 の座標を x=5.0 m, y=0.0 m と教える。誤差の計算に使う。|

もしこのコマンドを使ったら、``out``というディレクトリ直下に``shots.csv``ができるはずです。
そして同時に```out/model_fit.png```もできます。

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


### 着弾したい距離から速度を求める(solve)
```bash
python3 launch-model.py solve --model out/model.json --distance 4.5 --voltage 18.5
```
| パラメータ | 説明 |
| :--- | :---: |
|```--model out/model.json```|手順2で作ったモデルを読み込む|
|```--distance 4.5```|着弾させたい位置を指定|
|```voltage 18.5```|今のバッテリー電圧 [V]。省略すると、学習時の中央値を使う|
出力は、必要な実速度と、ロボットに指令すべき目標速度です。

### そのほかのコマンド


