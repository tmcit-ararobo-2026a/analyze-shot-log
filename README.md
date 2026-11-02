## analyze-shot-log
ベルト直動のログを解析するリポジトリです。

## 前準備GA
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

