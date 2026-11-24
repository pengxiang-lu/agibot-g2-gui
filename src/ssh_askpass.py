#!/usr/bin/python3
"""Qt SSH password/host-key prompt. Credentials remain in memory only."""
import sys
from PyQt5 import QtWidgets
app = QtWidgets.QApplication(sys.argv[:1])
app.setStyleSheet('QWidget { font-size: 20px; color: #173653; } QDialog { background: #eaf5ff; } QPushButton { padding: 8px 16px; } QLineEdit { padding: 7px; }')
prompt = sys.argv[1] if len(sys.argv)>1 else 'SSH 密码：'
if 'yes/no' in prompt.lower() or 'authenticity of host' in prompt.lower():
    result = QtWidgets.QMessageBox.question(None, '确认 SSH 主机指纹', prompt,
                 QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
    answer = 'yes' if result == QtWidgets.QMessageBox.Yes else 'no'
else:
    answer, ok = QtWidgets.QInputDialog.getText(None,'机器人 SSH 验证',prompt,QtWidgets.QLineEdit.Password)
    if not ok: sys.exit(1)
print(answer)
