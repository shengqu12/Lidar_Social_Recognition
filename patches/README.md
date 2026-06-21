# Patches

Jetson 系统级补丁（目标文件位于 Jetson 的 /opt/ros/... ，不在本 repo 内）。

## fix_rosbridge_imu.py
修补 rosbridge 的 message_conversion.py / subscribe.py，使其能正确序列化
PointCloud2 的 bytes data 字段（否则在 msg_instance_type_repr 抛 IndexError，
消息被 rosbridge 内部静默丢弃，订阅端 callback 永不触发）。

### 已应用记录
- node1 (172.26.42.167): 2026-06-11 已打补丁（message_conversion.py + subscribe.py）
- node3 (172.26.165.5): 2026-06-21 从 node1 拷贝 message_conversion.py
  (md5 61c9231ec39c0690d8289446c8fca3cc)；Jetson 上备份为
  message_conversion.py.bak_pre_patch_20260621 (原始 md5 599e5b34...)。
  注：node3 的 subscribe.py（cbor-raw 压缩路径）尚未补 —— 当前 JSON tracking
  不需要；若日后用 Foxglove 压缩订阅再补。

## inspect_bg_npz.py
调试工具（非补丁）：查看背景模型 .npz 的体素数 / 均值 / 标准差内容。
