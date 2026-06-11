import pandas as pd
import numpy as np

# ============================================================
# 1. 读取ground truth (groups.dat)
# ============================================================
def load_ground_truth(filepath):
    """读取groups.dat，只保留type=1的真实社交互动对"""
    gt_pairs = set()
    with open(filepath) as f:
        for line in f:
            parts = line.strip().split()
            person_id = int(parts[0])
            partner_id = int(parts[2])
            
            # 找interaction_type
            interaction_type = None
            for p in parts[3:]:
                if p in ('0', '1'):
                    interaction_type = int(p)
                    break
            
            if interaction_type == 1:
                # 用frozenset去方向性（A,B和B,A是同一对）
                pair = frozenset([person_id, partner_id])
                gt_pairs.add(pair)
    
    return gt_pairs

# ============================================================
# 2. 读取你的算法输出 (encounters_raw.csv)
# ============================================================
def load_detections(filepath):
    """读取碰撞检测结果"""
    df = pd.read_csv(filepath)
    detected_pairs = set()
    for _, row in df.iterrows():
        pair = frozenset([int(row['person1']), int(row['person2'])])
        detected_pairs.add(pair)
    return detected_pairs

# ============================================================
# 3. 计算Precision / Recall / F1
# ============================================================
def evaluate(detected_pairs, gt_pairs):
    tp = detected_pairs & gt_pairs      # 检测到且ground truth有
    fp = detected_pairs - gt_pairs      # 检测到但ground truth没有
    fn = gt_pairs - detected_pairs      # ground truth有但没检测到

    precision = len(tp) / (len(tp) + len(fp)) if detected_pairs else 0
    recall    = len(tp) / (len(tp) + len(fn)) if gt_pairs else 0
    f1        = 2 * precision * recall / (precision + recall) if (precision + recall) else 0

    print("=" * 40)
    print(f"检测到的pair数:       {len(detected_pairs)}")
    print(f"Ground truth pair数:  {len(gt_pairs)}")
    print(f"True Positive:        {len(tp)}")
    print(f"False Positive:       {len(fp)}")
    print(f"False Negative:       {len(fn)}")
    print("-" * 40)
    print(f"Precision:  {precision:.3f}")
    print(f"Recall:     {recall:.3f}")
    print(f"F1 Score:   {f1:.3f}")
    print("=" * 40)
    print(f"\nFlack论文基准: Precision=0.861")
    
    return precision, recall, f1

# ============================================================
# 4. 主程序
# ============================================================
if __name__ == '__main__':
    print("加载ground truth...")
    gt_pairs = load_ground_truth('./dataset/ATC_dataset/groups_ATC-1.dat')
    print(f"GT社交互动对: {len(gt_pairs)}")

    print("加载检测结果...")
    detected_pairs = load_detections('pipeline/encounters_raw.csv')
    print(f"算法检测对: {len(detected_pairs)}")

    print("\n评估结果:")
    evaluate(detected_pairs, gt_pairs)
