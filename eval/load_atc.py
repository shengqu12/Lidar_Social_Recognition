import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# load data
df = pd.read_csv(
    '../dataset/ATC_dataset/person_ATC-1_1000.csv',
    header=None,
    names=['timestamp', 'person_id', 'x', 'y', 'z', 'velocity', 'angle1', 'angle2']
)

# unit conversion: millimeters -> meters
df['x'] = df['x'] / 1000.0
df['y'] = df['y'] / 1000.0

print(f"Total frames: {df['timestamp'].nunique()}")
print(f"Total persons: {df['person_id'].nunique()}")
print(f"Time range: {df['timestamp'].min():.0f} ~ {df['timestamp'].max():.0f}")
print(f"\nFirst 5 rows:")
print(df.head())

# pick the most frequent person to inspect their trajectory
sample_id = df['person_id'].value_counts().index[0]
traj = df[df['person_id'] == sample_id].sort_values('timestamp')
print(f"\nMost frequent person ID={sample_id}, {len(traj)} frames total")

# visualize all persons' positions in the first frame
frame = df[df['timestamp'] == df['timestamp'].iloc[0]]
plt.figure(figsize=(10, 8))
plt.scatter(frame['x'], frame['y'], s=10, alpha=0.6)
plt.title('ATC first frame - all pedestrian positions')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.axis('equal')
plt.savefig('atc_first_frame.png')
print("\nSaved atc_first_frame.png")
