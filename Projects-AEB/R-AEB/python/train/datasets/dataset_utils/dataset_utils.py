import torch


def to_device(data, device):
    """
    将输入数据迁移到指定的设备，只递归处理 dict，Tensor 直接迁移，其它类型原样返回。
    Args:
        data: 可能是 Tensor、dict，或其它任意类型。
        device: 目标设备，可以是字符串（如 'cuda'、'cpu'）或 torch.device 实例。
    Returns:
        迁移到目标设备后的数据（原地修改 dict 中的 Tensor）。
    """
    # 统一将 device 转为 torch.device
    if not isinstance(device, torch.device):
        device = torch.device(device)

    # 如果是 Tensor，直接迁移
    if isinstance(data, torch.Tensor):
        return data.to(device)
    # 如果是字典，就对每个 value 递归处理
    elif isinstance(data, dict):
        for key, val in data.items():
            data[key] = to_device(val, device)
        return data
    # 其它类型（包括 list/tuple）直接返回，不作处理
    else:
        return data

