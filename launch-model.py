#!/usr/bin/env python3
"""launch_model.py  --  ベルト直動の着弾予測ツール (trials.jsonl -> CSV -> モデル -> 予測)

目的: ロボットの自動化のため「何 m/s で撃てばどこに落ちるか」「ここに落としたいなら何 m/s か」を
      ログから求められるようにする。

サブコマンド
  convert     jsonl -> CSV。min/max 速度で絞り込み、速度の低い順に並べ、的に対する誤差も計算
  fit         CSV から着弾モデルを作る (電圧・電流も候補に入れ、LOO交差検証で最良を自動選択)
  predict     速度(や指令値)・電圧から着弾位置を予測
  solve       着弾したい距離から、必要な速度と「指令すべき目標速度」を逆算
  table       速度 -> 着弾位置の表をCSVで出力
  export-cpp  ファームに組み込める C++ ヘッダ (補間テーブル) を出力

座標系: landing_position / cx,cy,cz と同じ座標系 (x=前方, y=横, z=高さ [m])。
        発射位置が原点でない場合は --launcher-x で「発射位置からの距離」に換算できる。

使い方の例
  python3 launch_model.py convert trials.jsonl -o out --vmin 3.0 --vmax 7.75 \\
          --target 4:5.0,0.0 --target 5:4.0,0.5,0.58
  python3 launch_model.py fit out/shots.csv --model-out out/model.json
  python3 launch_model.py predict --model out/model.json --velocity 6.0 --voltage 18.5
  python3 launch_model.py solve   --model out/model.json --distance 5.0 --voltage 18.5
  python3 launch_model.py table   --model out/model.json -o out/table.csv
  python3 launch_model.py export-cpp --model out/model.json -o out/launch_table.hpp
"""
import argparse
import itertools
import json
import os
import sys

import numpy as np
import pandas as pd

G = 9.81
DEFAULT_VMIN = 3.00
DEFAULT_VMAX = 7.75


# ======================================================================
# 読み込み
# ======================================================================
def load_jsonl(paths):
    """jsonl を読み、shot_id が戻った(または別ファイルになった)所でセッションを区切る。"""
    rows, sess = [], -1
    for p in paths:
        prev = None
        sess += 1
        with open(p, encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                r = json.loads(line)
                if prev is not None and r['shot_id'] <= prev:
                    sess += 1
                r['_session'] = sess
                r['_file'] = os.path.basename(p)
                prev = r['shot_id']
                rows.append(r)
    return rows


# ======================================================================
# ボール軌道の復元 (cx,cy,cz の時系列 -> 放物線)
# ======================================================================
def extract_track(samples, t_min=0.05):
    """cx,cy,cz が更新された点だけを取り出す。発射前(t<=t_min)の古い値は捨てる。"""
    out, last = [], None
    for s in samples:
        p = (s['cx'], s['cy'], s['cz'])
        if s['t'] > t_min and p != last:
            out.append((s['t'], *p))
        last = p
    return np.array(out) if out else np.empty((0, 4))


def _fit_lin(T, P):
    """x=a+bt, y=e+ft, z=c+dt-0.5gt^2 (g固定) を最小二乗で当てる。"""
    A = np.c_[np.ones_like(T), T]
    cx = np.linalg.lstsq(A, P[:, 0], rcond=None)[0]
    cy = np.linalg.lstsq(A, P[:, 1], rcond=None)[0]
    cz = np.linalg.lstsq(A, P[:, 2] + 0.5 * G * T ** 2, rcond=None)[0]
    return cx, cy, cz


def _pred(T, m):
    cx, cy, cz = m
    return np.c_[cx[0] + cx[1] * T, cy[0] + cy[1] * T, cz[0] + cz[1] * T - 0.5 * G * T ** 2]


def fit_flight(tr, thr=0.18, min_inliers=8):
    """RANSACで放物線を当てる。壁・的に当たった後の点や追跡ミスは外れ値として落ちる。"""
    if len(tr) < min_inliers:
        return None
    T, P = tr[:, 0], tr[:, 1:]
    best = None
    for i, j in itertools.combinations(range(len(T)), 2):
        if T[j] - T[i] < 0.1:
            continue
        m = _fit_lin(T[[i, j]], P[[i, j]])
        inl = np.linalg.norm(_pred(T, m) - P, axis=1) < thr
        if best is None or inl.sum() > best[0].sum():
            best = (inl, m)
    if best is None or best[0].sum() < min_inliers:
        return None
    inl, m = best
    for _ in range(3):
        m = _fit_lin(T[inl], P[inl])
        inl = np.linalg.norm(_pred(T, m) - P, axis=1) < thr
    if inl.sum() < min_inliers:
        return None
    d = np.linalg.norm(_pred(T, m) - P, axis=1)
    t0 = T[inl][0]
    cx, cy, cz = m
    vx, vy, vz = cx[1], cy[1], cz[1] - G * t0
    return {
        'model': m, 'n_inliers': int(inl.sum()), 'n_points': len(T),
        'rms': float(np.sqrt((d[inl] ** 2).mean())),
        'vx': vx, 'vy': vy, 'vz': vz,
        'speed': float(np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)),
        'angle': float(np.degrees(np.arctan2(vz, np.hypot(vx, vy)))),
        'release_z': float(cz[0] + cz[1] * t0 - 0.5 * G * t0 ** 2),
        't0': float(t0),
    }


def landing_at(fit, z_land):
    """放物線が高さ z_land を (下向きに) 横切る位置 (x, y)。届かなければ NaN。"""
    cx, cy, cz = fit['model']
    a, b, c = -0.5 * G, cz[1], cz[0] - z_land
    disc = b * b - 4 * a * c
    if disc < 0:
        return np.nan, np.nan
    t = (-b - np.sqrt(disc)) / (2 * a)      # 大きい方の根 = 下降中
    if t <= fit['t0']:
        return np.nan, np.nan
    return cx[0] + cx[1] * t, cy[0] + cy[1] * t


# ======================================================================
# convert
# ======================================================================
def parse_targets(specs):
    """'4:5.0,0.0' または '4:5.0,0.0,0.58' -> {4: (x, y, z or None)}"""
    out = {}
    for s in specs or []:
        tid, xyz = s.split(':')
        v = [float(a) for a in xyz.split(',')]
        out[int(tid)] = (v[0], v[1], v[2] if len(v) > 2 else None)
    return out


def build_shots(rows, args):
    targets = parse_targets(args.target)
    prev_land, prev_ts, out = {}, {}, []
    for r in rows:
        ss = r['samples']
        t = np.array([s['t'] for s in ss])
        va = np.array([s['v_act'] for s in ss])
        vt = np.array([s['v_tgt'] for s in ss])
        vo = np.array([s['volt'] for s in ss])
        has_cmd = 'v_cmd' in ss[0]
        vc = np.array([s.get('v_cmd', np.nan) for s in ss])

        k = int(np.argmax(va))
        peak = float(va[k])
        fired = peak > 0.5
        # v_tgt は発射後にランプする記録があるので「発射前〜ピーク時刻」の最大値を目標とする
        tgt = float(vt[t <= (t[k] if fired else 0.5)].max())
        cmd_max = float(np.nanmax(vc)) if has_cmd else np.nan
        if fired:
            reason = ''
        elif has_cmd:
            reason = 'cmd_not_received' if cmd_max < 0.1 else 'belt_not_moving'
        else:
            reason = 'no_motion'

        sid = r['_session']
        L = r['landing_position']
        key = None if L is None else (round(L['x'], 4), round(L['y'], 4), round(L['z'], 4))
        stale = L is not None and (r['trajectory_points_count'] == 0 or key == prev_land.get(sid))
        prev_land[sid] = key
        obs_ok = L is not None and not stale

        ts_now = pd.Timestamp(r['timestamp'])
        gap = (ts_now - prev_ts[sid]).total_seconds() if sid in prev_ts else np.nan
        prev_ts[sid] = ts_now

        # 的の位置: --target があればそれを、なければ jsonl の値
        tg = r['target']
        tx, ty, tz = targets.get(tg['target_id'], (tg['x'], tg['y'], None))
        tz_eff = tz if tz is not None else args.landing_z

        # 軌道復元 -> 的の高さでの到達位置
        fl = None
        if fired and not args.no_track:
            fl = fit_flight(extract_track(ss))
            if fl is not None and not (fl['vx'] > 1.0 and abs(fl['angle']) < 80 and fl['rms'] < 0.12):
                fl = None        # 静止物を追っている/発射前の値などは不採用
        px = py = np.nan
        if fl is not None:
            px, py = landing_at(fl, tz_eff)
        track_ok = bool(fl is not None and np.isfinite(px))

        row = {
            'session': sid, 'shot_id': r['shot_id'], 'timestamp': r['timestamp'],
            'gap_from_prev_s': gap,
            'target_id': tg['target_id'],
            'target_velocity': tgt, 'peak_velocity': peak,
            'achieved_ratio': peak / tgt if (fired and tgt > 0) else np.nan,
            'peak_time_s': float(t[k]) if fired else np.nan,
            'cmd_max': cmd_max,
            'max_current': r['summary']['max_current'],
            'avg_voltage': r['summary']['avg_voltage'],
            'min_voltage': float(vo[vo > 0].min()) if (vo > 0).any() else np.nan,
            'fired': fired, 'misfire_reason': reason,
            # --- 的 ---
            'target_x': tx, 'target_y': ty, 'target_z': tz_eff,
            # --- 観測された着弾(システムが記録した位置) ---
            'obs_valid': bool(obs_ok),
            'obs_x': L['x'] if obs_ok else np.nan,
            'obs_y': L['y'] if obs_ok else np.nan,
            'obs_z': L['z'] if obs_ok else np.nan,
            # --- 軌道から復元した発射条件と、的の高さでの到達位置 ---
            'track_ok': track_ok,
            'track_inliers': fl['n_inliers'] if fl else np.nan,
            'track_rms': fl['rms'] if fl else np.nan,
            'launch_speed': fl['speed'] if fl else np.nan,
            'launch_angle_deg': fl['angle'] if fl else np.nan,
            'release_z': fl['release_z'] if fl else np.nan,
            'pred_x': px if track_ok else np.nan,
            'pred_y': py if track_ok else np.nan,
        }
        # --- 学習用ラベル: 的の高さ付近で観測された着弾を優先、なければ軌道復元の値 ---
        if obs_ok and L['z'] <= tz_eff + 0.15:
            lx, ly, lsrc = L['x'], L['y'], 'observed'
        elif track_ok:
            lx, ly, lsrc = px, py, 'track'
        else:
            lx, ly, lsrc = np.nan, np.nan, ''
        row.update(label_x=lx, label_y=ly, label_source=lsrc)
        # --- 的に対する誤差 (+dx = 的より遠く、+dy = 的より y 正側) ---
        for tag, (ax, ay) in {'obs': (row['obs_x'], row['obs_y']), 'pred': (px, py),
                              'label': (lx, ly)}.items():
            if np.isfinite(ax):
                row[f'err_{tag}_dx'] = ax - tx
                row[f'err_{tag}_dy'] = ay - ty
                row[f'err_{tag}_dist'] = float(np.hypot(ax - tx, ay - ty))
            else:
                row[f'err_{tag}_dx'] = row[f'err_{tag}_dy'] = row[f'err_{tag}_dist'] = np.nan
        out.append(row)
    return pd.DataFrame(out)


def cmd_convert(args):
    rows = load_jsonl(args.inputs)
    os.makedirs(args.out, exist_ok=True)
    df = build_shots(rows, args)

    key = 'target_velocity' if args.sort_by == 'target' else 'peak_velocity'
    other = 'peak_velocity' if key == 'target_velocity' else 'target_velocity'
    df['in_range'] = (df[key] >= args.vmin) & (df[key] <= args.vmax)
    n_all = len(df)
    n_mis = int((~df.fired).sum())
    if not args.keep_misfires:
        df = df[df.fired]
    n_oor = int((~df.in_range).sum())
    if not args.include_all:
        df = df[df.in_range]
    df = df.sort_values([key, other, 'session', 'shot_id']).reset_index(drop=True)   # 低い -> 高い
    path = os.path.join(args.out, 'shots.csv')
    df.to_csv(path, index=False)

    if args.samples:
        sm = []
        for r in rows:
            for s in r['samples']:
                sm.append({'session': r['_session'], 'shot_id': r['shot_id'], **s})
        pd.DataFrame(sm).to_csv(os.path.join(args.out, 'samples.csv'), index=False)

    print(f'入力 {n_all} 発 -> 不発 {n_mis} 発' + ('(CSVに残す)' if args.keep_misfires else '(除外)')
          + f' / 速度範囲外({args.sort_by} が {args.vmin}〜{args.vmax} m/s の外) {n_oor} 発'
          + ('(印だけ付けて残す)' if args.include_all else '(除外)'))
    print(f'書き出し: {path}  ({len(df)} 行, 速度の低い順)')
    mis = df[~df.fired] if args.keep_misfires else None
    print(f'軌道を復元できた発数: {int(df.track_ok.sum())} / {len(df)}')
    if mis is not None and len(mis):
        print('不発の内訳:', mis.misfire_reason.value_counts().to_dict())
    tids = sorted(df.target_id.unique())
    if not args.target:
        print('注意: --target 未指定のため、的の座標は jsonl の値(全部 0,0)です。'
              ' 誤差(err_*)は「原点からのずれ」になります。 例: --target 4:5.0,0.0')
    else:
        miss = [int(t) for t in tids if t not in parse_targets(args.target)]
        if miss:
            print(f'注意: target_id {miss} の座標は未指定です(jsonl の値を使用)')
    e = df.err_obs_dist.dropna()
    if len(e):
        print(f'観測着弾の誤差: n={len(e)} 平均 {e.mean():.2f} m / 中央値 {e.median():.2f} m')
    e = df.err_pred_dist.dropna()
    if len(e):
        print(f'軌道復元での誤差: n={len(e)} 平均 {e.mean():.2f} m / 中央値 {e.median():.2f} m')
    nl = int(df.label_x.notna().sum())
    print(f'学習に使えるラベル(床の観測 or 軌道復元)がある発: {nl} / {len(df)}'
          + ('' if args.include_all else '  ※範囲外を除外中。fit で全部使うなら --include-all を付けて再変換'))


# ======================================================================
# モデル (リッジ回帰 + LOO交差検証)
# ======================================================================
TERM_SETS = [['v'], ['v', 'v2'], ['v', 'V'], ['v', 'I'], ['v', 'v2', 'V', 'I']]


def make_X(terms, v, V, I):
    v = np.atleast_1d(np.asarray(v, float))
    cols = {'v': v, 'v2': v ** 2,
            'V': np.broadcast_to(np.asarray(V, float), v.shape),
            'I': np.broadcast_to(np.asarray(I, float), v.shape)}
    return np.stack([cols[t] for t in terms], axis=1)


def ridge_fit(X, y, lam):
    mu, sd = X.mean(0), X.std(0)
    sd[sd < 1e-9] = 1.0
    A = np.c_[np.ones(len(X)), (X - mu) / sd]
    reg = lam * np.eye(A.shape[1])
    reg[0, 0] = 0.0
    beta = np.linalg.solve(A.T @ A + reg, A.T @ y)
    return {'mu': mu, 'sd': sd, 'beta': beta}


def ridge_predict(m, X):
    return np.c_[np.ones(len(X)), (X - m['mu']) / m['sd']] @ m['beta']


def loo_rmse(X, y, lam):
    errs = []
    for i in range(len(y)):
        keep = np.arange(len(y)) != i
        m = ridge_fit(X[keep], y[keep], lam)
        errs.append(ridge_predict(m, X[i:i + 1])[0] - y[i])
    return float(np.sqrt(np.mean(np.square(errs)))), np.array(errs)


def cmd_fit(args):
    df = pd.read_csv(args.csv)
    allfired = df[df.fired]
    d = df[df.fired & np.isfinite(df.label_x)].copy()
    if not args.use_all:
        d = d[d.in_range]
    d = d[np.isfinite(d.peak_velocity) & np.isfinite(d.avg_voltage)]
    n = len(d)
    print(f'学習に使える発数(発射済み・着弾ラベルあり' + ('' if args.use_all else '・速度範囲内') + f'): {n}'
          f'  (内訳 {d.label_source.value_counts().to_dict()})')
    if n < 4:
        sys.exit('データ不足です(最低4発、できれば20発以上)。\n'
                 '  - convert を --include-all 付きで実行し、fit に --use-all を付けると範囲外の発も使えます\n'
                 '  - カメラが動いていたか(trajectory_points が 0 の発が多くないか)確認してください')

    y = d.label_x.values
    Vref = float(d.avg_voltage.median())
    Iref = float(d.max_current.median())

    # --- 候補ごとに LOO 誤差を比べる ---
    results = []
    for terms in TERM_SETS:
        p = len(terms) + 1
        if n < p + 2:
            continue
        X = make_X(terms, d.peak_velocity.values, d.avg_voltage.values, d.max_current.values)
        rm, errs = loo_rmse(X, y, args.ridge)
        results.append((rm, terms, errs))
    best = min(results, key=lambda a: a[0])
    # 僅差(5%以内)なら項の少ない方を選ぶ
    simple = [a for a in results if a[0] <= best[0] * 1.05]
    rm, terms, errs = min(simple, key=lambda a: len(a[1]))

    X = make_X(terms, d.peak_velocity.values, d.avg_voltage.values, d.max_current.values)
    m = ridge_fit(X, y, args.ridge)
    resid = y - ridge_predict(m, X)
    dof = max(n - (len(terms) + 1), 1)
    sigma = float(np.sqrt((resid ** 2).sum() / dof))

    print('\n候補モデルの LOO(1つ抜き)誤差 [m]:')
    for r_, t_, _ in results:
        print(f"  {'*' if t_ == terms else ' '} {'+'.join(t_):<10s} LOO RMSE = {r_:.3f}")
    print(f'\n採用: {"+".join(terms)}   学習誤差σ={sigma:.3f} m   LOO RMSE={rm:.3f} m')

    # --- 指令値 -> 実速度 の比 (電圧の影響も見る) ---
    q = allfired[(allfired.target_velocity > 0) & (allfired.target_velocity <= args.vmax - 0.5)]
    q = q[np.isfinite(q.achieved_ratio) & np.isfinite(q.avg_voltage)]
    ratio = {'type': 'const', 'r0': float(q.achieved_ratio.median()) if len(q) else 1.0,
             'r1': 0.0, 'Vref': Vref, 'n': int(len(q))}
    if len(q) >= 8 and q.avg_voltage.std() > 0.3:
        c = np.corrcoef(q.avg_voltage, q.achieved_ratio)[0, 1]
        if abs(c) > 0.3:
            r1, r0 = np.polyfit(q.avg_voltage - Vref, q.achieved_ratio, 1)
            ratio.update(type='linear', r0=float(r0), r1=float(r1))
    print(f"指令→実速度の比: {ratio['type']}  r0={ratio['r0']:.3f}  r1={ratio['r1']:+.4f} /V  (n={ratio['n']})")

    # --- カバレッジ(どの速度帯にデータがあるか) ---
    lo = np.floor(min(args.vmin, d.peak_velocity.min()))
    hi = max(args.vmax, d.peak_velocity.max()) + 0.5
    edges = np.arange(lo, hi, 0.5)
    cnt = np.histogram(d.peak_velocity, bins=edges)[0]
    empty = [f'{edges[i]:.1f}〜{edges[i + 1]:.1f}' for i, c in enumerate(cnt) if c == 0]
    print('\n速度帯ごとの学習データ数 [m/s]:')
    print('  ' + '  '.join(f'{edges[i]:.1f}:{c}' for i, c in enumerate(cnt)))
    warn = []
    if n < 20:
        warn.append(f'学習データが {n} 発しかありません(20発以上を推奨)')
    if empty:
        warn.append('データが無い速度帯: ' + ', '.join(empty))
    if rm > 0.5:
        warn.append(f'LOO誤差が {rm:.2f} m と大きく、実用精度ではありません')
    for w in warn:
        print('警告:', w)

    model = {
        'terms': terms, 'mu': m['mu'].tolist(), 'sd': m['sd'].tolist(), 'beta': m['beta'].tolist(),
        'sigma': sigma, 'loo_rmse': rm, 'n': n, 'ridge': args.ridge,
        'Vref': Vref, 'Iref': Iref,
        'v_train_min': float(d.peak_velocity.min()), 'v_train_max': float(d.peak_velocity.max()),
        'v_ceiling': float(allfired.peak_velocity.quantile(0.98)),
        'vmin': args.vmin, 'vmax': args.vmax,
        'lateral_mean': float(d.label_y.mean()), 'lateral_std': float(d.label_y.std()),
        'ratio': ratio, 'warnings': warn,
    }
    with open(args.model_out, 'w', encoding='utf-8') as f:
        json.dump(model, f, indent=2, ensure_ascii=False)
    print(f'\nモデル保存: {args.model_out}')

    if not args.no_plot:
        plot_fit(d, model, m, errs, os.path.splitext(args.model_out)[0] + '_fit.png')


def plot_fit(d, model, m, loo_err, png):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    vs = np.linspace(model['vmin'], model['vmax'], 200)
    f = predict_x(model, vs, model['Vref'], model['Iref'])
    ax[0].plot(vs, f, 'k-', label=f"model ({'+'.join(model['terms'])})")
    ax[0].fill_between(vs, f - 1.645 * model['loo_rmse'], f + 1.645 * model['loo_rmse'],
                       color='gray', alpha=0.2, label='90% (from LOO RMSE)')
    ax[0].scatter(d.peak_velocity, d.label_x, c=d.session, cmap='tab10', s=45, zorder=3)
    for _, r in d.iterrows():
        ax[0].annotate(f"S{int(r.session)}#{int(r.shot_id)}", (r.peak_velocity, r.label_x),
                       textcoords='offset points', xytext=(4, 4), fontsize=7)
    ax[0].axvspan(model['v_train_min'], model['v_train_max'], color='tab:blue', alpha=0.06)
    ax[0].set_xlabel('belt velocity (peak) [m/s]')
    ax[0].set_ylabel('landing x [m] (observed or track-extrapolated)')
    ax[0].set_title(f"n={model['n']}  sigma={model['sigma']:.2f} m  LOO RMSE={model['loo_rmse']:.2f} m")
    ax[0].legend(); ax[0].grid(True)
    ax[1].bar(range(len(loo_err)), loo_err, color='tab:orange')
    ax[1].axhline(0, color='k', lw=1)
    ax[1].set_xlabel('training shot (sorted by velocity)')
    ax[1].set_ylabel('leave-one-out error [m]')
    ax[1].set_title('LOO residuals'); ax[1].grid(True, axis='y')
    fig.tight_layout()
    fig.savefig(png, dpi=110)
    print(f'図保存: {png}')


# ======================================================================
# 予測・逆算
# ======================================================================
def load_model(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def predict_x(model, v, V=None, I=None):
    V = model['Vref'] if V is None else V
    I = model['Iref'] if I is None else I
    X = make_X(model['terms'], v, V, I)
    m = {'mu': np.array(model['mu']), 'sd': np.array(model['sd']), 'beta': np.array(model['beta'])}
    return ridge_predict(m, X)


def ratio_at(model, V):
    r = model['ratio']
    return r['r0'] + r['r1'] * (V - r['Vref'])


def _range_warnings(model, v):
    w = []
    if v < model['v_train_min'] - 1e-9 or v > model['v_train_max'] + 1e-9:
        w.append(f"{v:.2f} m/s は学習範囲({model['v_train_min']:.2f}〜{model['v_train_max']:.2f})の外です(外挿)")
    if v > model['v_ceiling'] + 1e-9:
        w.append(f"{v:.2f} m/s は実測の上限(約{model['v_ceiling']:.2f} m/s)を超えています")
    return w


def cmd_predict(args):
    model = load_model(args.model)
    V = args.voltage if args.voltage is not None else model['Vref']
    I = args.current if args.current is not None else model['Iref']
    if args.velocity is not None:
        v, src = args.velocity, '実速度'
    else:
        v, src = args.command * ratio_at(model, V), f'指令 {args.command:.2f} m/s x 比'
    x = float(predict_x(model, v, V, I)[0])
    s = model['loo_rmse']
    print(f'入力: 速度={v:.3f} m/s ({src}) 電圧={V:.2f} V 電流={I:.1f} A')
    print(f"予測着弾 x = {x - args.launcher_x:.2f} m  (90%範囲 ±{1.645 * s:.2f} m / LOO誤差 {s:.2f} m)")
    print(f"横ずれ y ≈ {model['lateral_mean']:+.2f} m (ばらつき σ={model['lateral_std']:.2f} m)")
    for w in _range_warnings(model, v) + model.get('warnings', []):
        print('警告:', w)


def cmd_solve(args):
    model = load_model(args.model)
    V = args.voltage if args.voltage is not None else model['Vref']
    I = args.current if args.current is not None else model['Iref']
    goal = args.distance + args.launcher_x
    vs = np.arange(model['vmin'], model['vmax'] + 1e-9, 0.005)
    f = predict_x(model, vs, V, I)
    d = f - goal
    idx = np.where(np.sign(d[:-1]) * np.sign(d[1:]) <= 0)[0]
    if len(idx) == 0:
        j = int(np.argmin(np.abs(d)))
        print(f'目標 {args.distance:.2f} m には届きません。範囲内で最も近いのは '
              f'{vs[j]:.2f} m/s で着弾 {f[j] - args.launcher_x:.2f} m '
              f'(可能範囲 {f.min() - args.launcher_x:.2f}〜{f.max() - args.launcher_x:.2f} m)')
        return
    sols = []
    for i in idx:       # 線形補間で交点を求める
        t = d[i] / (d[i] - d[i + 1]) if d[i] != d[i + 1] else 0.0
        sols.append(vs[i] + t * (vs[i + 1] - vs[i]))
    v = min(sols)       # 複数あれば一番遅い(省エネ・安定)方
    r = ratio_at(model, V)
    cmd = v / r
    cmd_c = float(np.clip(cmd, model['vmin'], model['vmax']))
    print(f'目標着弾 {args.distance:.2f} m  (電圧 {V:.2f} V)')
    print(f'  必要な実速度     : {v:.3f} m/s')
    print(f'  指令すべき目標速度: {cmd:.3f} m/s  (実速度/指令 = {r:.3f})'
          + (f' -> 範囲内に丸めると {cmd_c:.2f}' if abs(cmd_c - cmd) > 1e-6 else ''))
    print(f"  予測誤差の目安   : ±{1.645 * model['loo_rmse']:.2f} m (90%)")
    if len(sols) > 1:
        print('  他の解(速度):', ', '.join(f'{s:.2f}' for s in sorted(sols) if s != v), '(モデルが単調でない)')
    for w in _range_warnings(model, v) + model.get('warnings', []):
        print('警告:', w)


def cmd_table(args):
    model = load_model(args.model)
    V = args.voltage if args.voltage is not None else model['Vref']
    I = args.current if args.current is not None else model['Iref']
    vs = np.arange(model['vmin'], model['vmax'] + 1e-9, args.step)
    x = predict_x(model, vs, V, I) - args.launcher_x
    r = ratio_at(model, V)
    pd.DataFrame({
        'belt_velocity': vs, 'command_velocity': vs / r, 'landing_x': x,
        'band90': 1.645 * model['loo_rmse'],
        'in_train_range': (vs >= model['v_train_min']) & (vs <= model['v_train_max']),
    }).to_csv(args.out, index=False)
    print(f'書き出し: {args.out}  ({len(vs)} 行, 電圧 {V:.2f} V)')


def raw_coefs(model):
    """標準化されたリッジ係数 -> 生の変数に対する係数 (x = c0 + cv*v + cv2*v^2 + cV*V + cI*I)"""
    mu, sd, b = np.array(model['mu']), np.array(model['sd']), np.array(model['beta'])
    coef = {'c0': float(b[0] - np.sum(b[1:] * mu / sd)), 'v': 0.0, 'v2': 0.0, 'V': 0.0, 'I': 0.0}
    for t, ci in zip(model['terms'], b[1:] / sd):
        coef[t] = float(ci)
    return coef


def export_cpp_formula(args, model):
    c = raw_coefs(model)
    r = model['ratio']
    txt = f'''// 自動生成: launch_model.py export-cpp --style formula  (手で編集しない)
// 学習 {model['n']} 発 / LOO誤差 {model['loo_rmse']:.2f} m / 採用項: {'+'.join(model['terms'])}
// 着弾位置 x [m] = c0 + cv*v + cv2*v^2 + cV*V + cI*I
//   v: 実ベルト速度[m/s]  V: バッテリー電圧[V]  I: ピーク電流[A]
#pragma once

namespace launch_model {{
constexpr float kC0  = {c['c0']:.8e}f;
constexpr float kCv  = {c['v']:.8e}f;
constexpr float kCv2 = {c['v2']:.8e}f;
constexpr float kCV  = {c['V']:.8e}f;
constexpr float kCI  = {c['I']:.8e}f;
constexpr float kVmin = {model['vmin']:.3f}f;     // 速度の下限 [m/s]
constexpr float kVmax = {model['vmax']:.3f}f;     // 速度の上限 [m/s]
constexpr float kVref = {model['Vref']:.3f}f;     // 電圧の既定値 [V] (学習時の中央値)
constexpr float kIref = {model['Iref']:.3f}f;     // 電流の既定値 [A]
constexpr float kTrainMin = {model['v_train_min']:.3f}f;   // 学習した速度範囲 (外は外挿)
constexpr float kTrainMax = {model['v_train_max']:.3f}f;
constexpr float kErrorBand90 = {1.645 * model['loo_rmse']:.3f}f;   // 予測の90%範囲 [m]
constexpr float kRatioR0 = {r['r0']:.5f}f;       // 実速度 / 指令速度
constexpr float kRatioR1 = {r['r1']:.5f}f;       // 電圧1Vあたりの比の変化
constexpr float kRatioVref = {r['Vref']:.3f}f;

/** 実速度 [m/s] -> 着弾位置 [m] */
inline float distance_from_velocity(float v, float voltage = kVref, float current = kIref) {{
    return kC0 + kCv * v + kCv2 * v * v + kCV * voltage + kCI * current;
}}

/** 着弾したい位置 [m] -> 必要な実速度 [m/s]  (範囲を走査し、最も遅い解を返す。届かなければ端の速度) */
inline float velocity_from_distance(float d, float voltage = kVref, float current = kIref) {{
    constexpr float kStep = 0.01f;
    float prev_v = kVmin;
    float prev = distance_from_velocity(prev_v, voltage, current) - d;
    if (prev >= 0.0f) return kVmin;
    for (float v = kVmin + kStep; v <= kVmax + 1e-6f; v += kStep) {{
        const float cur = distance_from_velocity(v, voltage, current) - d;
        if (cur >= 0.0f) {{
            return prev_v + prev / (prev - cur) * (v - prev_v);
        }}
        prev_v = v;
        prev = cur;
    }}
    return kVmax;
}}

/** 実速度 -> 指令すべき目標速度 (バッテリー電圧 [V] で補正) */
inline float command_velocity(float v, float voltage = kVref) {{
    return v / (kRatioR0 + kRatioR1 * (voltage - kRatioVref));
}}
}}  // namespace launch_model
'''
    with open(args.out, 'w', encoding='utf-8') as f:
        f.write(txt)
    print(f'書き出し: {args.out}  (式: x = {c["c0"]:.4g} + {c["v"]:.4g}*v + {c["v2"]:.4g}*v^2'
          f' + {c["V"]:.4g}*V + {c["I"]:.4g}*I)')


def cmd_export_cpp(args):
    model = load_model(args.model)
    if args.style == 'formula':
        return export_cpp_formula(args, model)
    V = args.voltage if args.voltage is not None else model['Vref']
    I = model['Iref']
    vs = np.arange(model['vmin'], model['vmax'] + 1e-9, args.step)
    x = predict_x(model, vs, V, I) - args.launcher_x
    if np.any(np.diff(x) <= 0):
        print('警告: 速度に対して着弾が単調増加ではありません。逆引き用に単調化します(データ不足の可能性)')
        x = np.maximum.accumulate(x + np.arange(len(x)) * 1e-4)
    fl = lambda a: ', '.join(f'{v:.4f}f' for v in a)
    r = model['ratio']
    txt = f'''// 自動生成: launch_model.py export-cpp  (手で編集しない)
// 基準電圧 {V:.2f} V / 学習 {model['n']} 発 / LOO誤差 {model['loo_rmse']:.2f} m
// 距離は座標系の x [m]。学習範囲: {model['v_train_min']:.2f}〜{model['v_train_max']:.2f} m/s
#pragma once
#include <cstddef>

namespace launch_model {{
constexpr std::size_t kSize = {len(vs)};
constexpr float kVelocity[kSize] = {{{fl(vs)}}};      // 実速度 [m/s]
constexpr float kDistance[kSize] = {{{fl(x)}}};       // 着弾位置 [m] (単調増加)
constexpr float kRatioR0 = {r['r0']:.5f}f;            // 実速度 / 指令速度
constexpr float kRatioR1 = {r['r1']:.5f}f;            // 電圧1Vあたりの比の変化
constexpr float kRatioVref = {r['Vref']:.3f}f;
constexpr float kTrainMin = {model['v_train_min']:.3f}f;
constexpr float kTrainMax = {model['v_train_max']:.3f}f;
constexpr float kErrorBand90 = {1.645 * model['loo_rmse']:.3f}f;   // 予測の90%範囲 [m]

inline float interp(const float* xs, const float* ys, float x) {{
    if (x <= xs[0]) return ys[0];
    if (x >= xs[kSize - 1]) return ys[kSize - 1];
    std::size_t i = 0;
    while (xs[i + 1] < x) ++i;
    const float t = (x - xs[i]) / (xs[i + 1] - xs[i]);
    return ys[i] + t * (ys[i + 1] - ys[i]);
}}
/** 実速度 [m/s] -> 着弾位置 [m] */
inline float distance_from_velocity(float v) {{ return interp(kVelocity, kDistance, v); }}
/** 着弾したい位置 [m] -> 必要な実速度 [m/s] */
inline float velocity_from_distance(float d) {{ return interp(kDistance, kVelocity, d); }}
/** 実速度 -> 指令すべき目標速度 (バッテリー電圧 [V] で補正) */
inline float command_velocity(float v, float voltage) {{
    return v / (kRatioR0 + kRatioR1 * (voltage - kRatioVref));
}}
}}  // namespace launch_model
'''
    with open(args.out, 'w', encoding='utf-8') as f:
        f.write(txt)
    print(f'書き出し: {args.out}')


# ======================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest='cmd', required=True)

    c = sp.add_parser('convert', help='jsonl -> CSV (速度範囲で絞り込み・低い順に並べ替え・誤差計算)')
    c.add_argument('inputs', nargs='+')
    c.add_argument('-o', '--out', default='out')
    c.add_argument('--vmin', type=float, default=DEFAULT_VMIN, help='最小速度 [m/s] (既定 3.00)')
    c.add_argument('--vmax', type=float, default=DEFAULT_VMAX, help='最大速度 [m/s] (既定 7.75)')
    c.add_argument('--sort-by', choices=['target', 'peak'], default='target',
                   help='範囲判定と並べ替えに使う速度 (target=指令した目標速度 / peak=実測ピーク)')
    c.add_argument('--include-all', action='store_true', help='範囲外も残す(in_range 列で区別)')
    c.add_argument('--keep-misfires', action='store_true', help='不発も残す(fired=False)')
    c.add_argument('--target', action='append', metavar='ID:X,Y[,Z]',
                   help='的の座標 [m]。複数指定可。例 --target 4:5.0,0.0 --target 5:4.0,0.5,0.58')
    c.add_argument('--landing-z', type=float, default=0.10,
                   help='軌道から着弾位置を求める高さ [m] (的に Z 指定が無いとき。既定 0.10=床)')
    c.add_argument('--no-track', action='store_true', help='軌道復元を省略(高速)')
    c.add_argument('--samples', action='store_true', help='時系列も samples.csv に書く')
    c.set_defaults(func=cmd_convert)

    f = sp.add_parser('fit', help='CSV から着弾モデルを作る')
    f.add_argument('csv')
    f.add_argument('--model-out', default='out/model.json')
    f.add_argument('--vmin', type=float, default=DEFAULT_VMIN)
    f.add_argument('--vmax', type=float, default=DEFAULT_VMAX)
    f.add_argument('--ridge', type=float, default=0.01, help='リッジ正則化の強さ')
    f.add_argument('--use-all', action='store_true',
                   help='速度範囲外(in_range=False)の発も学習に使う(convert --include-all が必要)')
    f.add_argument('--no-plot', action='store_true')
    f.set_defaults(func=cmd_fit)

    def common(p):
        p.add_argument('--model', required=True)
        p.add_argument('--voltage', type=float, help='バッテリー電圧 [V] (省略=学習時の中央値)')
        p.add_argument('--current', type=float, help='ピーク電流 [A] (省略=学習時の中央値)')
        p.add_argument('--launcher-x', type=float, default=0.0, help='発射位置の x [m] (距離をここ基準にする)')

    p = sp.add_parser('predict', help='速度 -> 着弾位置')
    common(p)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument('--velocity', type=float, help='実際のベルト速度 [m/s]')
    g.add_argument('--command', type=float, help='指令する目標速度 [m/s]')
    p.set_defaults(func=cmd_predict)

    p = sp.add_parser('solve', help='着弾したい距離 -> 必要な速度・指令値')
    common(p)
    p.add_argument('--distance', type=float, required=True, help='着弾させたい位置 [m]')
    p.set_defaults(func=cmd_solve)

    p = sp.add_parser('table', help='速度 -> 着弾位置の表')
    common(p)
    p.add_argument('-o', '--out', default='out/table.csv')
    p.add_argument('--step', type=float, default=0.25)
    p.set_defaults(func=cmd_table)

    p = sp.add_parser('export-cpp', help='ファーム用 C++ ヘッダを出力')
    common(p)
    p.add_argument('-o', '--out', default='out/launch_table.hpp')
    p.add_argument('--step', type=float, default=0.25, help='(table のとき)速度の刻み幅')
    p.add_argument('--style', choices=['table', 'formula'], default='table',
                   help='table=補間テーブル(既定) / formula=式の係数')
    p.set_defaults(func=cmd_export_cpp)

    args = ap.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()