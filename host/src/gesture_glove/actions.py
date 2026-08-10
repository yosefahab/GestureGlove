"""Translation of recognised gestures into keyboard actions."""

from typing import Protocol

from .config import GestureProfile

IDLE_GESTURE = "idle"
COMBO_SEPARATOR = "+"


class Keyboard(Protocol):
    """The slice of pyautogui this module uses, so a fake can be substituted.

    Arguments are positional only so the protocol matches pyautogui, whose own
    parameter names differ from the ones used here.
    """

    def press(self, key: str, /) -> None: ...

    def hotkey(self, *keys: str) -> None: ...


def resolve_binding(
    profile: GestureProfile, gesture: str, previous: str
) -> str | None:
    """Return the shortcut to fire for a gesture, or None if it should be ignored.

    Repeats are suppressed because the firmware re-reports the same label on
    every inference window; firing twice would cancel or double the action.
    """
    if gesture == previous or gesture == IDLE_GESTURE:
        return None
    return profile.bindings.get(gesture)


def perform(binding: str, keyboard: Keyboard) -> None:
    """Send a single key, or a combination written as 'ctrl+tab'."""
    if COMBO_SEPARATOR in binding:
        keyboard.hotkey(*binding.split(COMBO_SEPARATOR))
    else:
        keyboard.press(binding)


# Key names accepted in config.json, from pyautogui. Combine with '+'.
# ['\t', '\n', '\r', ' ', '!', '"', '#', '$', '%', '&', "'", '(', ')',
# '*', '+', ',', '-', '.', '/', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
# ':', ';', '<', '=', '>', '?', '@', '[', '\\', ']', '^', '_', '`',
# 'a' .. 'z', '{', '|', '}', '~', 'accept', 'add', 'alt', 'altleft', 'altright',
# 'apps', 'backspace', 'browserback', 'browserfavorites', 'browserforward',
# 'browserhome', 'browserrefresh', 'browsersearch', 'browserstop', 'capslock',
# 'clear', 'convert', 'ctrl', 'ctrlleft', 'ctrlright', 'decimal', 'del', 'delete',
# 'divide', 'down', 'end', 'enter', 'esc', 'escape', 'execute', 'f1' .. 'f24',
# 'final', 'fn', 'hanguel', 'hangul', 'hanja', 'help', 'home', 'insert', 'junja',
# 'kana', 'kanji', 'launchapp1', 'launchapp2', 'launchmail', 'launchmediaselect',
# 'left', 'modechange', 'multiply', 'nexttrack', 'nonconvert', 'num0' .. 'num9',
# 'numlock', 'pagedown', 'pageup', 'pause', 'pgdn', 'pgup', 'playpause',
# 'prevtrack', 'print', 'printscreen', 'prntscrn', 'prtsc', 'prtscr', 'return',
# 'right', 'scrolllock', 'select', 'separator', 'shift', 'shiftleft', 'shiftright',
# 'sleep', 'space', 'stop', 'subtract', 'tab', 'up', 'volumedown', 'volumemute',
# 'volumeup', 'win', 'winleft', 'winright', 'yen', 'command', 'option',
# 'optionleft', 'optionright']
