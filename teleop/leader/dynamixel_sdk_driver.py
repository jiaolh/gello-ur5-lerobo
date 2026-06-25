import time
from pathlib import Path
from typing import Sequence

import numpy as np
import dynamixel_sdk.port_handler as port_handler_module
from dynamixel_sdk.group_sync_read import GroupSyncRead
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.robotis_def import COMM_SUCCESS


PROTOCOL_VERSION = 2.0
DEFAULT_BAUDRATE = 57600
USB_LOW_LATENCY_TIMER_MS = 1
SDK_PACKET_LATENCY_TIMER_MS = 16

ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4

TORQUE_DISABLE = 0

DEFAULT_GELLO_IDS = (1, 2, 3, 4, 5, 6, 7)


class DynamixelSDKDriver:
    def __init__(
        self,
        port: str,
        ids: Sequence[int] = DEFAULT_GELLO_IDS,
        baudrate: int = DEFAULT_BAUDRATE,
        max_read_retries: int = 3,
    ) -> None:
        self.port = port
        self.ids = tuple(ids)
        self.baudrate = baudrate
        self.max_read_retries = max_read_retries
        self._last_positions_by_id: dict[int, float] = {}
        self._sync_readers: dict[tuple[int, ...], GroupSyncRead] = {}

        # USB latency 可以降到 1ms，但 SDK 包超时要保守一些，避免 57600 波特率下误判超时。
        port_handler_module.LATENCY_TIMER = SDK_PACKET_LATENCY_TIMER_MS
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        self._open_port()
        self._set_usb_low_latency()
        self._get_sync_reader(self.ids)
        self.disable_torque()

    def _open_port(self) -> None:
        """打开串口并设置波特率。"""
        if not self.port_handler.openPort():
            raise RuntimeError(f"打开 Dynamixel 串口失败: {self.port}")

        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"设置 Dynamixel 波特率失败: {self.baudrate}")

    def _set_usb_low_latency(self) -> None:
        """尽量降低 FTDI/USB 串口 latency timer，不能写入时保持原配置。"""
        tty_name = Path(self.port).resolve().name
        latency_path = Path("/sys/class/tty") / tty_name / "device" / "latency_timer"
        if not latency_path.exists():
            return

        try:
            current_latency = int(latency_path.read_text(encoding="utf-8").strip())
            if current_latency <= USB_LOW_LATENCY_TIMER_MS:
                return
            latency_path.write_text(f"{USB_LOW_LATENCY_TIMER_MS}\n", encoding="utf-8")
            print(f"Dynamixel 串口 latency_timer 已设置为 {USB_LOW_LATENCY_TIMER_MS} ms")
        except PermissionError:
            print(
                "warning: 无权限设置 Dynamixel 串口 latency_timer；"
                f"可手动执行: echo {USB_LOW_LATENCY_TIMER_MS} | sudo tee {latency_path}"
            )
        except Exception as exc:
            print(f"warning: 设置 Dynamixel 串口 latency_timer 失败: {exc}")

    def _get_sync_reader(self, ids: Sequence[int]) -> GroupSyncRead:
        """按 ID 组合缓存同步读取器，避免每次重建参数列表。"""
        read_ids = tuple(int(dxl_id) for dxl_id in ids)
        if read_ids in self._sync_readers:
            return self._sync_readers[read_ids]

        sync_reader = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )
        for dxl_id in read_ids:
            if not sync_reader.addParam(dxl_id):
                raise RuntimeError(f"添加 Dynamixel 同步读取 ID 失败: {dxl_id}")
        self._sync_readers[read_ids] = sync_reader
        return sync_reader

    def disable_torque(self) -> None:
        """关闭电机扭矩，让 GELLO 主手可以被操作者自由拖动。"""
        for dxl_id in self.ids:
            comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_DISABLE,
            )
            if comm_result != COMM_SUCCESS or dxl_error != 0:
                raise RuntimeError(
                    f"关闭 Dynamixel 扭矩失败: id={dxl_id}, "
                    f"comm_result={comm_result}, dxl_error={dxl_error}"
                )

    def read_positions(self, ids: Sequence[int] | None = None) -> np.ndarray:
        """读取所有电机当前位置，单位是弧度。"""
        read_ids = self.ids if ids is None else tuple(int(dxl_id) for dxl_id in ids)
        sync_reader = self._get_sync_reader(read_ids)
        comm_result = None
        for _ in range(self.max_read_retries):
            comm_result = sync_reader.txRxPacket()
            if comm_result == COMM_SUCCESS:
                break
            time.sleep(0.001)

        if comm_result != COMM_SUCCESS:
            if all(dxl_id in self._last_positions_by_id for dxl_id in read_ids):
                print(f"warning: Dynamixel 同步读取失败，使用上一帧数据: {comm_result}")
                return np.array(
                    [self._last_positions_by_id[dxl_id] for dxl_id in read_ids],
                    dtype=float,
                )
            raise RuntimeError(f"Dynamixel 同步读取失败: comm_result={comm_result}")

        positions = []
        for dxl_id in read_ids:
            if not sync_reader.isAvailable(
                dxl_id,
                ADDR_PRESENT_POSITION,
                LEN_PRESENT_POSITION,
            ):
                raise RuntimeError(f"Dynamixel 位置数据不可用: id={dxl_id}")

            raw_position = sync_reader.getData(
                dxl_id,
                ADDR_PRESENT_POSITION,
                LEN_PRESENT_POSITION,
            )
            position = self._raw_position_to_radians(raw_position)
            self._last_positions_by_id[dxl_id] = position
            positions.append(position)

        result = np.array(positions, dtype=float)
        return result.copy()

    def close(self) -> None:
        """关闭串口连接。"""
        self.port_handler.closePort()

    @staticmethod
    def _raw_position_to_radians(raw_position: int) -> float:
        """将 Dynamixel 原始位置 tick 转成弧度。"""
        # Present Position 是 32 位补码，超过正数范围时需要转成负数。
        if raw_position > 0x7FFFFFFF:
            raw_position -= 0x100000000

        return raw_position / 2048.0 * np.pi