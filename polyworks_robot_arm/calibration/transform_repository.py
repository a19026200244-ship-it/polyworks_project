"""变换保存/加载仓库。"""

from __future__ import annotations

import json
from pathlib import Path

from polyworks_robot_arm.calibration.transform_models import RigidTransform
from polyworks_robot_arm.common.config import TRANSFORM_REPOSITORY_FILE
from polyworks_robot_arm.common.exceptions import RepositoryError

# 仓库层的作用可以简单理解成:
# “把标定结果安全地存起来，再稳定地读出来”。
# 这样 controller 和 UI 就不用关心 JSON 文件格式细节。


class TransformRepository:
    """负责把标定结果保存到本地 JSON 文件。"""

    def __init__(self, file_path: Path = TRANSFORM_REPOSITORY_FILE) -> None:
        self.file_path = Path(file_path)

    # transforms.json 里存的是字典列表，这里统一恢复成 dataclass。
    def list_transforms(self) -> list[RigidTransform]:
        """列出全部已保存变换。"""
        payload = self._load_payload()
        transforms = [RigidTransform.from_dict(item) for item in payload.get("transforms", [])]
        return sorted(transforms, key=lambda item: item.created_at, reverse=True)

    def get_transform(self, name: str) -> RigidTransform | None:
        """按名称读取单个变换。"""
        for transform in self.list_transforms():
            if transform.name == name:
                return transform
        return None

    def save_transform(self, transform: RigidTransform, set_active: bool = False) -> None:
        """保存或覆盖一个变换。"""
        payload = self._load_payload()
        transforms = payload.get("transforms", [])

        updated = False
        for index, item in enumerate(transforms):
            if item.get("name") == transform.name:
                # 同名时视为覆盖更新，方便重复标定同一个变换名。
                transforms[index] = transform.to_dict()
                updated = True
                break

        if not updated:
            # 不同名则直接追加，形成历史记录。
            transforms.append(transform.to_dict())

        payload["transforms"] = transforms
        if set_active:
            # active_name 代表“当前默认用于结果回传的变换”。
            payload["active_name"] = transform.name

        self._save_payload(payload)

    def set_active_transform(self, name: str | None) -> None:
        """设置当前激活变换。"""
        payload = self._load_payload()
        if name is None:
            payload["active_name"] = None
            self._save_payload(payload)
            return

        if self.get_transform(name) is None:
            raise RepositoryError(f"未找到名为 {name} 的标定变换")

        payload["active_name"] = name
        self._save_payload(payload)

    def get_active_transform_name(self) -> str | None:
        """获取激活变换名称。"""
        payload = self._load_payload()
        active_name = payload.get("active_name")
        return None if active_name in {"", None} else str(active_name)

    def get_active_transform(self) -> RigidTransform | None:
        """读取当前激活变换。"""
        active_name = self.get_active_transform_name()
        if active_name is None:
            return None
        return self.get_transform(active_name)

    def _load_payload(self) -> dict:
        """读取 JSON 文件内容。"""
        if not self.file_path.exists():
            # 第一次运行时文件还不存在，这里返回一个空仓库结构。
            return {"active_name": None, "transforms": []}

        try:
            return json.loads(self.file_path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise RepositoryError(f"读取标定文件失败: {exc}") from exc

    def _save_payload(self, payload: dict) -> None:
        """写回 JSON 文件。"""
        try:
            # 先确保目录存在，再把中文友好的 JSON 写回去。
            self.file_path.parent.mkdir(parents=True, exist_ok=True)
            self.file_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except Exception as exc:
            raise RepositoryError(f"保存标定文件失败: {exc}") from exc