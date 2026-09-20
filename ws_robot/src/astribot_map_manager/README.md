# astribot_map_manager

后续已补 C++ Voxel 受管加载适配器，最新能力及验证边界见 [适配器交付说明](../../../docs/P2_VOXEL_SESSION_ADAPTER_20260919.md)；本文原批次记录保留。

C++ 地图资产归档、不可变工位版本和人工换层事务。服务复用 OperatorCommand，正常控制经 operator_backend 的租约入口。

[接口、配置、验证与阶段边界](../../../docs/P2_MAP_STATION_TRANSACTIONS_20260919.md)。
默认目录为 /tmp，部署必须配置持久化目录。生产切图适配器未接入时仅提供归档/工位功能；不模拟实际加载成功。
