"""PolyWorks COM 通信封装。

这个模块只做一件事：
负责 Python 和 PolyWorks Inspector 之间的 COM 调用。
"""

from __future__ import annotations

from pathlib import Path

try:
    import ctypes
    import comtypes
    import comtypes.client

    HAS_COM = True
except ImportError:
    HAS_COM = False
    ctypes = None
    comtypes = None


if HAS_COM:
    # 下面这些接口定义来自 PolyWorks COM SDK。
    # 业务层只需要知道“连接”和“执行命令”，
    # 具体 COM 细节都收口在这里。
    class IIMCommandCenter(comtypes.IUnknown):
        _iid_ = comtypes.GUID("{214c846e-fbfd-11d6-9394-00b0d0224d3a}")
        _methods_ = [
            comtypes.COMMETHOD(
                [],
                ctypes.HRESULT,
                "CommandExecute",
                (["in"], ctypes.c_wchar_p, "command"),
                (["out", "retval"], ctypes.POINTER(ctypes.c_long), "return_value"),
            ),
            comtypes.COMMETHOD(
                [],
                ctypes.HRESULT,
                "ReturnValueGetAsString",
                (["in"], ctypes.c_long, "return_code"),
                (["out", "retval"], ctypes.POINTER(comtypes.BSTR), "message"),
            ),
            comtypes.COMMETHOD(
                [],
                ctypes.HRESULT,
                "ReturnValueIsSuccess",
                (["in"], ctypes.c_long, "return_code"),
                (["out", "retval"], ctypes.POINTER(ctypes.c_long), "is_success"),
            ),
        ]


    class IIMInspectProject(comtypes.IUnknown):
        _iid_ = comtypes.GUID("{2d90e82a-1460-11d7-9397-00b0d0224d3a}")
        _methods_ = [
            comtypes.COMMETHOD(
                [],
                ctypes.HRESULT,
                "CommandCenterCreate",
                (
                    ["out"],
                    ctypes.POINTER(ctypes.POINTER(IIMCommandCenter)),
                    "command_center",
                ),
            ),
        ]


    class IIMInspect(comtypes.IUnknown):
        _iid_ = comtypes.GUID("{10976620-fc02-11d6-9394-00b0d0224d3a}")
        _methods_ = [
            comtypes.COMMETHOD(
                [],
                ctypes.HRESULT,
                "Login",
                (["in"], ctypes.c_wchar_p, "user_name"),
            ),
            comtypes.COMMETHOD(
                [],
                ctypes.HRESULT,
                "ProjectGetCurrent",
                (["out"], ctypes.POINTER(ctypes.POINTER(IIMInspectProject)), "project"),
            ),
        ]


class PolyWorksConnector:
    """通过 COM 接口与 PolyWorks Inspector 通信。"""

    PROG_ID = "InnovMetric.PolyWorks.IMInspect"

    def __init__(self) -> None:
        # 这些对象会在 connect() 后由 COM 返回
        self.inspect_obj = None
        self.iim_inspect = None
        self.project = None
        self.cmd_center = None
        self.connected = False

    def connect(self) -> None:
        """连接到已经安装好的 PolyWorks Inspector。

        连接策略分两步：
        1. 优先附着到用户已经打开的 Inspector 会话
        2. 如果没打开，再尝试新建一个 COM 对象
        """
        if not HAS_COM:
            raise RuntimeError("缺少 comtypes，请先运行: pip install comtypes")

        # 初始化当前线程的 COM 环境。
        comtypes.CoInitialize()

        try:
            # 优先附着到已经打开的 Inspector 会话。
            self.inspect_obj = comtypes.client.GetActiveObject(self.PROG_ID)
        except Exception:
            try:
                self.inspect_obj = comtypes.client.CreateObject(self.PROG_ID)
            except Exception as exc:
                raise RuntimeError(f"无法连接到 PolyWorks Inspector: {exc}") from exc

        # 下面这几步是在 COM 对象里拿到真正可调用的 Inspector 接口。
        self.iim_inspect = self.inspect_obj.QueryInterface(IIMInspect)
        self.project = self.iim_inspect.ProjectGetCurrent()
        self.cmd_center = self.project.CommandCenterCreate()
        self.connected = True

    def execute_command(self, command: str):
        """执行一条 PolyWorks 宏命令。

        PolyWorks 的很多二次开发能力，最终都会落到这里：
        Python 负责拼命令字符串，Inspector 负责真正执行。
        """
        if not self.connected:
            raise RuntimeError("尚未连接到 PolyWorks")

        result = self.cmd_center.CommandExecute(command)
        is_success = self.cmd_center.ReturnValueIsSuccess(result)

        if not is_success:
            message = self.cmd_center.ReturnValueGetAsString(result)
            raise RuntimeError(f"命令执行失败: {message}\n命令: {command}")

        return result

    def execute_macro(self, macro_path: str | Path) -> None:
        """执行一个临时生成的 PolyWorks 宏文件。"""
        self.execute_command(f'MACRO EXEC ( "{Path(macro_path).as_posix()}" )')

    def execute_commands(self, commands: list[str]) -> None:
        """按顺序执行多条 PolyWorks 命令。"""
        for command in commands:
            self.execute_command(command)

    def create_point(self, point: tuple[float, float, float], name: str) -> None:
        """在 PolyWorks 中创建一个点。

        这个辅助函数主要是为了减少业务层里重复写命令字符串。
        """
        x, y, z = point
        self.execute_command(
            f'FEATURE PRIMITIVE POINT CREATE ( {x}, {y}, {z}, "Nominal", "{name}", )'
        )

    def select_none(self) -> None:
        """清空当前对象选择。"""
        self.execute_command('TREEVIEW OBJECT SELECT NONE')

    def select_object(self, name: str, state: str = "On") -> None:
        """选择或取消选择一个对象。"""
        self.execute_command(f'TREEVIEW OBJECT SELECT ( "{name}", "{state}" )')

    def delete_object(self, name: str) -> None:
        """删除一个对象。"""
        self.execute_command(f'EDIT OBJECT DELETE ( "{name}" )')

    def disconnect(self) -> None:
        """断开 COM 连接，释放相关对象。"""
        self.cmd_center = None
        self.project = None
        self.iim_inspect = None
        self.inspect_obj = None
        self.connected = False

        if HAS_COM:
            try:
                # 与 CoInitialize 配对，避免 COM 资源没有正确释放。
                comtypes.CoUninitialize()
            except Exception:
                pass