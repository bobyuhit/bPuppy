"""
KittenBlock 代码上传工具
用法: python upload_kb.py COM3 program.py
  或 从标准输入: python upload_kb.py COM3 - < program.py
"""
import sys
import subprocess


def upload(port: str, filepath: str):
    """通过 mpremote 上传 user_prog.py 到 bPuppy"""
    if filepath == '-':
        # 从标准输入读取
        code = sys.stdin.read()
        # 写到临时文件
        import tempfile, os
        tmp = os.path.join(tempfile.gettempdir(), 'bpuppy_upload.py')
        with open(tmp, 'w') as f:
            f.write(code)
        filepath = tmp

    print(f'上传 {filepath} → bPuppy ({port})')
    subprocess.run([
        'mpremote', 'connect', port,
        'fs', 'cp', filepath, ':user_prog.py'
    ], check=True)
    print('上传完成！重启 bPuppy 生效。')


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    upload(sys.argv[1], sys.argv[2])
