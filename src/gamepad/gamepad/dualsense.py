# システム・前提
from typing import Optional, Union, Dict, Any

# 外部ライブラリ
from evdev import InputDevice, categorize, ecodes, list_devices, AbsInfo

class DualSense:
    def __init__(self, device_path: Optional[str] = None, deadzone: float = 0.05):
        """
        
        DualSenseコントローラと接続して状態を受け取り・出力するクラス
        
        Args:
            device_path (Optional[str]): コントローラのパス
            deadzone (float): スティックのデッドゾーン
            
        """
        self.path = device_path or self._find_dualsense_path()
        self.device = InputDevice(self.path)
        self.deadzone = deadzone
        
        # 軸レンジの取得
        abs_caps = self.device.capabilities().get(ecodes.EV_ABS, [])
        self.abs_info: Dict[int, AbsInfo] = {code: info for code, info in abs_caps}
            
        # 状態のバッファー
        self.state: Dict[str, Dict[str, Any]] = {
            "axis": {
                "lx": 0.0, "ly": 0.0,
                "rx": 0.0, "ry": 0.0,
                "l2": 0.0, "r2": 0.0,
                "dpx": 0, "dpy": 0,
            },
            "buttons": {
                "cross": False, "circle": False, "square": False, "triangle": False,
                "l1": False, "l2": False, "l3": False, "r1": False, "r2": False, "r3": False,
                "touch": False, "share": False, "option": False, "ps": False
            }
        }
        
        # ボタンとイベント番号のマップ
        self.btn_map = {
            304: "square",
            305: "cross",
            306: "circle",
            307: "triangle",
            308: "l1",
            310: "l2",
            314: "l3",
            309: "r1",
            311: "r2",
            315: "r3",
            317: "touch",
            312: "share",
            313: "option",
            316: "ps"   
        }
    
    def _find_dualsense_path(self) -> Union[str, None]:
        """
        
        DualSenseコントローラを検索する
        
        """
        # デバイスのリストからDualSenseコントローラを探す
        for path in list_devices():
            device = InputDevice(path)
            name = (device.name or "").lower()
            if ("dualsense" in name) or ("wireless controller" in name):
                return path
        raise RuntimeError("DualSense Wireless Controller is not found.")

    def _scale_center(self, code: int, val: float) -> float:
        """
        
        スティック状態のスケールを0.0センターに変換する
        
        Args:
            code (int): イベントコード
            val (float): 値
        
        Returns:
            (float): 変換後の値
        
        """
        info = self.abs_info[code]
        mn, mx = info.min, info.max
        md = (mn + mx) / 2.0
        rng = max(mx - md, md - mn) or 1.0
        return max(-1.0, min(1.0, (val - md) / rng))

    def _apply_deadzone(self, val: float) -> float:
        """
        
        デッドゾーンを適用する
        
        Args:
            val (float): 値
        
        Returns:
            (float): 適用後の値
        
        """
        return 0.0 if abs(val) < self.deadzone else val

    def _scale_01(self, code: int, val: float) -> float:
        """
        
        値を[0, 1]の範囲に変換する
        
        Args:
            code (int): イベントコード
            val (float): 値
        
        Returns:
            (float): 変換後の値
        
        """
        info = self.abs_info[code]
        rng = (info.max - info.min) or 1.0
        return max(0.0, min(1.0, (val - info.min) / rng))
    
    def read(self) -> Dict[str, Any]:
        """
        
        コントローラの現在の状態を読み取って出力する
        
        Returns:
            (Dict[str, Any]): 軸・ボタンの状態
        
        """
        
        while True:
            ev = self.device.read_one()
            
            if ev is None:
                break
                    
            # アナログ入力イベントを受け取った場合
            if ev.type == ecodes.EV_ABS:
                code, val = ev.code, ev.value
                if code == 0:
                    self.state["axis"]["lx"] = self._apply_deadzone(+self._scale_center(code, val))
                elif code == 1:
                    self.state["axis"]["ly"] = self._apply_deadzone(+self._scale_center(code, val) * -1.0)
                elif code == 2:
                    self.state["axis"]["rx"] = self._apply_deadzone(+self._scale_center(code, val))
                elif code == 5:
                    self.state["axis"]["ry"] = self._apply_deadzone(+self._scale_center(code, val) * -1.0)
                elif code == 3:
                    self.state["axis"]["l2"] = self._scale_01(code, val)
                elif code == 4:
                    self.state["axis"]["r2"] = self._scale_01(code, val)
                elif code == 16:
                    self.state["axis"]["dpx"] = int(max(-1, min(1, val)))
                elif code == 17:
                    self.state["axis"]["dpy"] = int(max(-1, min(1, -val)))
            
            # デジタル入力イベントを受け取った場合
            elif ev.type == ecodes.EV_KEY:
                name = self.btn_map.get(ev.code)
                if name is not None:
                    self.state["buttons"][name] = (ev.value != 0)
        
        return dict(self.state)
    
    def close(self):
        """
        
        コントローラとの通信を停止する
        
        """
        
        try:
            self.device.close()
        except Exception:
            pass
