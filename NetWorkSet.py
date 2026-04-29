import socket
import subprocess
import re
import platform
import ipaddress


def is_valid_ipv4(ip):
    try:
        ipaddress.IPv4Address(ip)
        return True
    except Exception:
        return False


def is_bad_local_ip(ip):
    """
    过滤明显不适合作为局域网通信地址的 IP
    """
    if not is_valid_ipv4(ip):
        return True

    ip_obj = ipaddress.IPv4Address(ip)

    # 回环地址
    if ip_obj.is_loopback:
        return True

    # 自动私有地址，常见于未正常联网
    if ip_obj in ipaddress.IPv4Network("169.254.0.0/16"):
        return True

    # 198.18.0.0/15 通常用于测试、虚拟网卡、代理 TUN 等场景
    if ip_obj in ipaddress.IPv4Network("198.18.0.0/15"):
        return True

    return False


def get_local_ip_by_target(target_ip):
    """
    根据目标设备 IP 获取本机用于访问该目标的 IP。
    例如 UR 机械臂是 192.168.3.100，则通常会返回 192.168.3.4。
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # UDP connect 不会真的发送数据，只是让系统选择路由
        s.connect((target_ip, 80))

        local_ip = s.getsockname()[0]
        s.close()

        if not is_bad_local_ip(local_ip):
            return local_ip

    except Exception:
        pass

    return ""


def get_local_ip_from_ipconfig():
    """
    Windows 下从 ipconfig 中解析 IPv4 地址，并尽量排除虚拟网卡。
    """
    try:
        output = subprocess.check_output(
            ["ipconfig"],
            encoding="gbk",
            errors="ignore"
        )

        blocks = re.split(r"\n(?=\S.*?:)", output)

        candidates = []

        for block in blocks:
            block_lower = block.lower()

            # 跳过明显的虚拟网卡、蓝牙、断开连接适配器
            bad_keywords = [
                "vmware",
                "virtualbox",
                "vethernet",
                "bluetooth",
                "蓝牙",
                "meta",
                "已断开连接",
                "disconnected",
            ]

            if any(k in block_lower for k in bad_keywords):
                continue

            ip_match = re.search(
                r"IPv4 地址.*?:\s*([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)",
                block
            )

            if ip_match:
                ip = ip_match.group(1)

                if not is_bad_local_ip(ip):
                    candidates.append(ip)

        if candidates:
            return candidates[0]

    except Exception:
        pass

    return ""


def get_local_ip_from_ifconfig():
    """
    Linux / macOS 下从 ifconfig 中解析 IPv4 地址，跳过虚拟网卡。
    """
    try:
        output = subprocess.check_output(
            ["ifconfig"],
            encoding="utf-8",
            errors="ignore"
        )

        blocks = re.split(r"\n(?=\S)", output)

        bad_keywords = [
            "meta",
            "vmware",
            "virtualbox",
            "vethernet",
            "bluetooth",
            "蓝牙",
            "docker",
            "veth",
            "br-",
            "virbr",
            "tun",
            "tap",
            "vnet",
            "tailscale",
            "zerotier",
            "wg",
            "lo:",
        ]

        candidates = []

        for block in blocks:
            block_lower = block.lower()

            if any(k in block_lower for k in bad_keywords):
                continue

            ip_match = re.search(
                r"inet\s+([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)",
                block
            )

            if ip_match:
                ip = ip_match.group(1)

                if not is_bad_local_ip(ip):
                    candidates.append(ip)

        if candidates:
            return candidates[0]

    except Exception:
        pass

    return ""


def get_local_ip(target_ip=None):
    """
    获取本机 IPv4 地址。

    推荐用法：
        get_local_ip("192.168.3.100")

    如果 target_ip 不为空，会优先获取访问该目标设备时使用的本机 IP。
    """

    # 最推荐：根据目标设备 IP 获取对应本机 IP
    if target_ip is not None:
        ip = get_local_ip_by_target(target_ip)
        if ip:
            return ip

    system_name = platform.system().lower()

    if system_name == "windows":
        ip = get_local_ip_from_ipconfig()
        if ip:
            return ip
    else:
        ip = get_local_ip_from_ifconfig()
        if ip:
            return ip

    return ""