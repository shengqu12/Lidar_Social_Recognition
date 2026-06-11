import pandas as pd
import numpy as np
from itertools import combinations

def load_atc(filepath):
    df = pd.read_csv(filepath, header=None,
        names=['timestamp', 'person_id', 'x', 'y', 'z', 
               'velocity', 'angle1', 'angle2'])
    df['x'] = df['x'] / 1000.0
    df['y'] = df['y'] / 1000.0
    df['velocity'] = df['velocity'] / 1000.0  # mm/s → m/s
    return df

def filter_moving_persons(df, min_displacement=1.0):
    """Step 1: Filter stationary targets"""
    person_disp = {}
    for pid, group in df.groupby('person_id'):
        x_range = group['x'].max() - group['x'].min()
        y_range = group['y'].max() - group['y'].min()
        displacement = np.sqrt(x_range**2 + y_range**2)
        person_disp[pid] = displacement
    moving = [pid for pid, d in person_disp.items() if d >= min_displacement]
    return df[df['person_id'].isin(moving)]

def filter_by_velocity(df, min_avg_velocity=0.1, max_avg_velocity=3.0):
    """
    Filter out false-stationary objects:
    - Average velocity too low (<0.1 m/s) -> stationary object
    - Average velocity too high (>3.0 m/s) -> outlier
    """
    valid_ids = []
    for pid, group in df.groupby('person_id'):
        avg_vel = group['velocity'].mean()
        if min_avg_velocity <= avg_vel <= max_avg_velocity:
            valid_ids.append(pid)
    print(f"  remaining after velocity filter: {len(valid_ids)} persons")
    return df[df['person_id'].isin(valid_ids)]

def filter_by_deceleration(df, encounters, decel_threshold=0.2):
    """
    Step 2: At least one person must show deceleration before the encounter
    """
    valid = []
    df_indexed = df.set_index(['person_id', 'timestamp']).sort_index()
    
    for _, row in encounters.iterrows():
        ts = row['timestamp']
        p1_id = int(row['person1'])
        p2_id = int(row['person2'])
        
        try:
            p1_traj = df[df['person_id']==p1_id].sort_values('timestamp')
            p2_traj = df[df['person_id']==p2_id].sort_values('timestamp')
            
            p1_before = p1_traj[p1_traj['timestamp'] <= ts].tail(10)
            p2_before = p2_traj[p2_traj['timestamp'] <= ts].tail(10)
            
            if len(p1_before) < 3 or len(p2_before) < 3:
                continue
            
            # velocity change: mean of second half minus mean of first half
            mid1 = len(p1_before) // 2
            mid2 = len(p2_before) // 2
            p1_decel = p1_before['velocity'].iloc[mid1:].mean() - p1_before['velocity'].iloc[:mid1].mean()
            p2_decel = p2_before['velocity'].iloc[mid2:].mean() - p2_before['velocity'].iloc[:mid2].mean()
            
            if p1_decel < -decel_threshold or p2_decel < -decel_threshold:
                valid.append(row)
        except:
            continue
    
    return pd.DataFrame(valid)

def detect_encounters(df, proximity=1.5, heading_threshold=90.0):
    """Step 3+4: proximity + heading detection"""
    encounters = []
    timestamps = sorted(df['timestamp'].unique())
    
    print(f"Processing {len(timestamps)} frames...")
    
    for i, ts in enumerate(timestamps):
        if i % 10000 == 0:
            print(f"  progress: {i}/{len(timestamps)}")
        
        frame = df[df['timestamp'] == ts]
        if len(frame) < 2:
            continue
        
        # pairwise check for all persons in the current frame
        persons = frame.set_index('person_id')
        ids = list(persons.index)
        
        for id1, id2 in combinations(ids, 2):
            p1 = persons.loc[id1]
            p2 = persons.loc[id2]
            
            # Step 3: distance check
            dist = np.sqrt((p1['x']-p2['x'])**2 + (p1['y']-p2['y'])**2)
            if dist > proximity:
                continue
            
            # Step 4: heading check (angle1 is the movement direction)
            angle_diff = abs(p1['angle1'] - p2['angle1'])
            angle_diff = min(angle_diff, 2*np.pi - angle_diff)
            angle_diff_deg = np.degrees(angle_diff)
            
            if angle_diff_deg >= heading_threshold:
                encounters.append({
                    'timestamp': ts,
                    'person1': id1,
                    'person2': id2,
                    'distance': dist,
                    'angle_diff': angle_diff_deg
                })
    
    return pd.DataFrame(encounters)

def deduplicate(encounters, time_window=3.0):
    """Step 5: Deduplicate — count the same pair only once within time_window seconds"""
    if len(encounters) == 0:
        return encounters
    
    encounters = encounters.sort_values('timestamp')
    keep = []
    seen = {}  # (id1, id2) -> last_timestamp
    
    for _, row in encounters.iterrows():
        pair = (min(row['person1'], row['person2']), 
                max(row['person1'], row['person2']))
        
        if pair not in seen or (row['timestamp'] - seen[pair]) > time_window:
            keep.append(row)
            seen[pair] = row['timestamp']
    
    return pd.DataFrame(keep)

def filter_artifact_ids(df, max_frame_ratio=0.3):
    """Step 6: Filter artifact IDs that appear in more than 30% of total frames"""
    total_frames = df['timestamp'].nunique()
    frame_counts = df.groupby('person_id')['timestamp'].nunique()
    bad_ids = set(frame_counts[frame_counts > total_frames * max_frame_ratio].index)
    print(f"  filtering artifact IDs: {bad_ids}")
    return df[~df['person_id'].isin(bad_ids)]

if __name__ == '__main__':
    print("Loading data...")
    df = load_atc('../dataset/ATC_dataset/person_ATC-1_1000.csv')
    
    print("Step 1: Filtering stationary targets...")
    df = filter_moving_persons(df)
    print(f"  remaining: {df['person_id'].nunique()} persons")

    print("Step 1b: Filtering artifacts...")
    df = filter_artifact_ids(df)
    print(f"  remaining: {df['person_id'].nunique()} persons")
    # # for speed, initially test on only the first 30 minutes of data
    # t_start = df['timestamp'].min()
    # t_end = t_start + 1800  # 30 minutes
    df_sample = df
    print(f"  whole day data: {df_sample['timestamp'].nunique()} frames, {df_sample['person_id'].nunique()} persons")

    print("Step 3+4: Detecting encounter events...")
    encounters = detect_encounters(df_sample)
    print(f"  raw encounter events: {len(encounters)}")

    print("Step 2: Deceleration filter...")
    encounters = filter_by_deceleration(df, encounters)
    print(f"  after deceleration filter: {len(encounters)}")

    print("Step 5: Deduplication...")
    encounters = deduplicate(encounters)
    print(f"  after deduplication: {len(encounters)} independent encounter events")

    # Step 6: save
    encounters.to_csv('encounters_raw.csv', index=False)
    print("Saved encounters_raw.csv")
    print(encounters.head(10))


    import matplotlib.pyplot as plt

    # map encounter events back to coordinates
    df_indexed = df_sample.set_index(['timestamp', 'person_id'])

    coords = []
    for _, row in encounters.iterrows():
        try:
            p1 = df_sample[(df_sample['timestamp']==row['timestamp']) & 
                        (df_sample['person_id']==row['person1'])].iloc[0]
            coords.append({'x': p1['x'], 'y': p1['y']})
        except:
            pass

    coords_df = pd.DataFrame(coords)

    plt.figure(figsize=(10, 8))
    plt.scatter(coords_df['x'], coords_df['y'], s=20, alpha=0.5, c='red')
    plt.title('Collision Heatmap - ATC (first 30min)')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.axis('equal')
    plt.savefig('collision_heatmap.png')
    print("Saved collision_heatmap.png")
    # inspect which pair encounters most frequently
    print(encounters.groupby(['person1','person2']).size().sort_values(ascending=False).head(10))